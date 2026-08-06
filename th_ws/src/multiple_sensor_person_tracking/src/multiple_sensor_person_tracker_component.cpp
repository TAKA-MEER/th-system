#include <cstdio>
#include <cmath>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>   // tf2::getYaw (ego-motion compensation)

#include <laser_geometry/laser_geometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"

#include "multiple_sensor_person_tracking/msg/following_position.hpp"
#include "multiple_sensor_person_tracking/msg/person_candidates.hpp"
#include "multiple_sensor_person_tracking/srv/select_target.hpp"

#include "multiple_observation_kalman_filter/multiple_observation_kalman_filter.hpp"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace multiple_sensor_person_tracking {
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    enum Status {
        NO_EXISTS = 0, EXISTS_LEG, EXISTS_BODY, EXISTS_LEG_AND_BODY
    };
    enum class DetectionMode {
        LEG,
        BODY,
        BODY_LEG
    };

    class PersonTracker : public rclcpp_lifecycle::LifecycleNode {
        private:
            rclcpp_lifecycle::LifecyclePublisher<multiple_sensor_person_tracking::msg::FollowingPosition>::SharedPtr pub_following_position_;
            rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
            rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacles_;
            rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_target_odom_;
            // Safety: tell the real robot to STOP when the target is lost or switched.
            rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr pub_stop_;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;
            // Live candidate list (for operator target selection) + explicit select service.
            rclcpp_lifecycle::LifecyclePublisher<multiple_sensor_person_tracking::msg::PersonCandidates>::SharedPtr pub_candidates_;
            rclcpp::Service<multiple_sensor_person_tracking::srv::SelectTarget>::SharedPtr srv_select_target_;
            multiple_sensor_person_tracking::msg::PersonCandidates last_candidates_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_nontravelable_region_;
            rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_dr_spaam_;
            rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr sub_image_;
            PointCloud::Ptr cloud_nontravelable_region_;

            std::unique_ptr<multiple_observation_kalman_filter::KalmanFilter> kf_;
            laser_geometry::LaserProjection projector_;
            pcl::KdTreeFLANN<PointT> flann_;
            pcl::ExtractIndices<PointT> extract_;
            pcl::RadiusOutlierRemoval<PointT> outrem_;
            pcl::VoxelGrid<PointT> voxel_;
            PointCloud::Ptr cloud_scan_;
            visualization_msgs::msg::MarkerArray::SharedPtr marker_array_;
            multiple_sensor_person_tracking::msg::FollowingPosition::SharedPtr following_position_;
            sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg_;
            geometry_msgs::msg::PoseArray::ConstSharedPtr dr_spaam_msg_;

            tf2_ros::Buffer tfBuffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_sub_;
            std::string target_frame_;
            std::string odom_frame_name_;
            std::string scan_frame_name_;

            geometry_msgs::msg::Point previous_target_;
            rclcpp::Time previous_time_;
            bool exists_target_;
            // Ego-motion compensation: pose of target_frame_ in odom_frame_name_ as of
            // the last processed detection, so the next one can undo the robot's own
            // movement before associating. See compensateEgoMotion().
            tf2::Transform previous_tracking_pose_;
            bool has_previous_tracking_pose_;
            double leg_tracking_range_;
            double body_tracking_range_;
            double target_range_;
            double target_cloud_radius_;
            bool display_marker_;
            double no_exists_time_;
            double target_change_tolerance_;
            // Methods A-D: recover a target that was lost behind an obstacle.
            geometry_msgs::msg::Point last_target_pos_; // last known/predicted target position at loss
            geometry_msgs::msg::Point last_target_vel_; // pre-occlusion velocity (x=vx, y=vy) at loss
            geometry_msgs::msg::Point previous_vel_;    // latest estimated velocity (x=vx, y=vy)
            double target_lost_time_;                   // wall time the target was dropped (-1 = not lost)
            double reacquire_tolerance_;                // how long to keep re-acquiring near last_target_pos_ [s]
            double reacquire_radius_rate_;              // gate growth per second of occlusion [m/s]
            double reacquire_lead_max_;                 // C: cap on velocity-projected emergence lead [m]
            double reacquire_vel_weight_;               // D: weight of velocity mismatch in re-acquire cost
            double coast_tolerance_max_;                // B: max coast time at rest before dropping [s]
            double coast_speed_scale_;                  // B: speed at which coast time is roughly halved [m/s]
            double attention_leg_time_;
            unsigned int attention_leg_idx_;
            bool merge_nontravelable_region_;
            DetectionMode detection_mode_;
            bool active_;
            // Safety-stop state: latched true when the target is (possibly) switched to a
            // different person; cleared by the reset service / on_activate. prev_stop_ is
            // used to log STOP/RESUME only on transitions.
            bool target_changed_latched_;
            bool prev_stop_;

            // Explicit-selection gating (furniture-leg false cold-acquire mitigation):
            // when true, cold-acquire (no target, no recent loss) never spontaneously
            // grabs the nearest leg -- it only runs within manual_seed_timeout_ seconds
            // of an operator calling ~/reset_tracking or ~/select_target, searching
            // around cold_acquire_seed_ instead of always the robot origin.
            bool require_explicit_target_selection_;
            geometry_msgs::msg::Point cold_acquire_seed_;
            double manual_seed_time_;
            double manual_seed_timeout_;

            visualization_msgs::msg::Marker makeLegPoseMarker( const std::vector<geometry_msgs::msg::Pose>& leg_poses );
            visualization_msgs::msg::Marker makeLegAreaMarker( const std::vector<geometry_msgs::msg::Pose>& leg_poses );
            visualization_msgs::msg::Marker makeBodyPoseMarker( const std::vector<vision_msgs::msg::Detection3D>& body_poses );
            visualization_msgs::msg::Marker makeTargetPoseMarker( const Eigen::Vector4f& target_pose );
            visualization_msgs::msg::Marker makeTargetTextMarker( const Eigen::Vector4f& target_pose );

            int findTwoObservationValue(
                const std::vector<geometry_msgs::msg::Pose>& leg_poses,
                const std::vector<vision_msgs::msg::Detection3D>& body_poses,
                Eigen::Vector2f* leg_observed_value,
                Eigen::Vector2f* body_observed_value );

            bool searchObstacles(
                const geometry_msgs::msg::Point& search_pt,
                const PointCloud::Ptr input_cloud,
                sensor_msgs::msg::PointCloud2* obstacles );

            geometry_msgs::msg::PointStamped transformPoint(
                const std::string& org_frame,
                const std::string& target_frame,
                const geometry_msgs::msg::Point& point );

            // Undo the robot's own movement since the previous detection, so that the
            // association gate and the constant-velocity prediction see only the
            // person's motion. Called once per detection frame.
            void compensateEgoMotion();

            void scan_callback (
                const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg );

            void nontravelableRegionCallback(
                const sensor_msgs::msg::PointCloud2::ConstSharedPtr& nontravelable_region_msg );

            void dr_spaam_callback (
                const geometry_msgs::msg::PoseArray::ConstSharedPtr &dr_spaam_msg
            );

            void callbackPoseArray (
                const vision_msgs::msg::Detection3DArray::ConstSharedPtr &body_msg );

            // Publish following_position_ together with the derived stop signal, and log
            // STOP/RESUME transitions to the terminal.
            void publishFollowing();

            // Reset the safety latch and force a fresh target acquisition.
            void resetTrackingCallback(
                const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response );

            // Force-seed the tracked position at an operator-chosen candidate/point.
            void selectTargetCallback(
                const std::shared_ptr<multiple_sensor_person_tracking::srv::SelectTarget::Request> request,
                std::shared_ptr<multiple_sensor_person_tracking::srv::SelectTarget::Response> response );
        public:
            explicit PersonTracker(const rclcpp::NodeOptions & options)
            : rclcpp_lifecycle::LifecycleNode("person_tracker", options),
            tfBuffer_(this->get_clock()),
            active_(false)
            {}

            CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
            CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
            CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);
            CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
            CallbackReturn on_error(const rclcpp_lifecycle::State & state);
            void resetInterfaces();
    };
}

visualization_msgs::msg::Marker multiple_sensor_person_tracking::PersonTracker::makeLegPoseMarker( const std::vector<geometry_msgs::msg::Pose>& leg_poses ) {
    visualization_msgs::msg::Marker leg_marker;
    leg_marker.header.frame_id = target_frame_;
    leg_marker.header.stamp = this->get_clock()->now();
    leg_marker.ns = "leg_marker";
    leg_marker.id =  1;
    leg_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    leg_marker.action = visualization_msgs::msg::Marker::ADD;
    leg_marker.scale.x = 0.15;leg_marker.scale.y = 0.15;leg_marker.scale.z = 0.15;
    leg_marker.color.r = 1.0; leg_marker.color.g = 0.0; leg_marker.color.b = 0.0; leg_marker.color.a = 1.0;
    leg_marker.pose.orientation.w = 1.0;
    leg_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    for ( const auto& pose : leg_poses ) leg_marker.points.push_back( pose.position );
    return leg_marker;
}

visualization_msgs::msg::Marker multiple_sensor_person_tracking::PersonTracker::makeLegAreaMarker( const std::vector<geometry_msgs::msg::Pose>& leg_poses ) {
    visualization_msgs::msg::Marker leg_marker;
    std::vector<double> offset_x, offset_y;
    double tolerance = 2.0*M_PI / 20.0;
    double radius = 0.4;
    for ( unsigned int i = 0; i < 20; i++ ) {
        offset_x.push_back( radius * std::cos(i * tolerance) );
        offset_y.push_back( radius * std::sin(i * tolerance) );
    }
    leg_marker.header.frame_id = target_frame_;
    leg_marker.header.stamp = this->get_clock()->now();
    leg_marker.ns = "leg_area_marker";
    leg_marker.id =  1;
    leg_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    leg_marker.action = visualization_msgs::msg::Marker::ADD;
    leg_marker.scale.x = 0.03;
    leg_marker.color.r = 1.0; leg_marker.color.g = 0.0; leg_marker.color.b = 0.0; leg_marker.color.a = 1.0;
    leg_marker.pose.orientation.w = 1.0;
    leg_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    for ( const auto& pose : leg_poses ) {
        for ( unsigned int i = 0; i < 20; i++ ) {
            geometry_msgs::msg::Point p0, p1;
            p0.x = pose.position.x + offset_x[i];
            p0.y = pose.position.y + offset_y[i];
            p0.z = -0.3;
            p1.x = ( i+1 < 20 ) ? pose.position.x + offset_x[i+1] : pose.position.x + offset_x[0];
            p1.y = ( i+1 < 20 ) ? pose.position.y + offset_y[i+1] : pose.position.y + offset_y[0];;
            p1.z = -0.3;
            leg_marker.points.push_back( p0 );
            leg_marker.points.push_back( p1 );
        }
    }
    return leg_marker;
}

visualization_msgs::msg::Marker multiple_sensor_person_tracking::PersonTracker::makeBodyPoseMarker( const std::vector<vision_msgs::msg::Detection3D>& body_poses ) {
    visualization_msgs::msg::Marker body_marker;
    body_marker.header.frame_id = target_frame_;
    body_marker.header.stamp = this->get_clock()->now();
    body_marker.ns = "body_marker";
    body_marker.id =  1;
    body_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    body_marker.action = visualization_msgs::msg::Marker::ADD;
    body_marker.scale.x = 0.15;body_marker.scale.y = 0.15;body_marker.scale.z = 0.15;
    body_marker.color.r = 0.0; body_marker.color.g = 1.0; body_marker.color.b = 0.0; body_marker.color.a = 1.0;
    body_marker.pose.orientation.w = 1.0;
    body_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    for ( const auto& detection : body_poses ) body_marker.points.push_back(detection.bbox.center.position);
    return body_marker;
}

visualization_msgs::msg::Marker multiple_sensor_person_tracking::PersonTracker::makeTargetPoseMarker( const Eigen::Vector4f& target_pose ) {
    visualization_msgs::msg::Marker target_marker;
    target_marker.header.frame_id = target_frame_;
    target_marker.header.stamp = this->get_clock()->now();
    target_marker.ns = "target_marker";
    target_marker.id =  1;
    target_marker.type = visualization_msgs::msg::Marker::ARROW;
    target_marker.action = visualization_msgs::msg::Marker::ADD;
    target_marker.scale.x = 0.5;target_marker.scale.y = 0.15;target_marker.scale.z = 0.15;
    target_marker.color.r = 1.0; target_marker.color.g = 1.0; target_marker.color.b = 0.0; target_marker.color.a = 1.0;
    target_marker.pose.position.x = target_pose[0];
    target_marker.pose.position.y = target_pose[1];
    target_marker.pose.position.z = 0.5;
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0, 0, std::atan2(target_pose[3], target_pose[2]));
    geometry_msgs::msg::Quaternion quat_msg;
    tf2::convert(quat_tf, quat_msg);
    target_marker.pose.orientation = quat_msg;
    target_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    return target_marker;
}

visualization_msgs::msg::Marker multiple_sensor_person_tracking::PersonTracker::makeTargetTextMarker( const Eigen::Vector4f& target_pose ) {
    // Text label showing the tracked target coordinate as seen from target_frame_
    // (= the LiDAR frame in a single-LiDAR setup).
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = target_frame_;
    text_marker.header.stamp = this->get_clock()->now();
    text_marker.ns = "target_text_marker";
    text_marker.id = 1;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.scale.z = 0.2; // text height [m]
    text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0; text_marker.color.a = 1.0;
    text_marker.pose.position.x = target_pose[0];
    text_marker.pose.position.y = target_pose[1];
    text_marker.pose.position.z = 0.9; // float the label above the target
    text_marker.pose.orientation.w = 1.0;
    char buf[64];
    std::snprintf( buf, sizeof(buf), "(%.2f, %.2f) m", target_pose[0], target_pose[1] );
    text_marker.text = buf;
    text_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    return text_marker;
}

int multiple_sensor_person_tracking::PersonTracker::findTwoObservationValue(
    const std::vector<geometry_msgs::msg::Pose>& leg_poses,
    const std::vector<vision_msgs::msg::Detection3D>& body_poses,
    Eigen::Vector2f* leg_observed_value,
    Eigen::Vector2f* body_observed_value )
{

    geometry_msgs::msg::Point search_pt, leg_pt, body_pt;

    const double now = this->get_clock()->now().seconds();

    // Decide WHERE and HOW WIDE to search for the tracked leg (Method A):
    //  - tracking     : around the last estimate; gate = leg_tracking_range_,
    //                   widened while the target is briefly occluded (coasting),
    //                   so a drifted prediction still re-associates on re-emergence.
    //  - re-acquire   : target was recently lost behind an obstacle -> search around
    //                   its last known position with a gate that grows with the
    //                   elapsed occlusion time, so the SAME person is picked up again.
    //  - cold acquire : no recent target -> nearest leg to the robot.
    double leg_min_distance;
    bool reacquire_mode = false;
    double lost_dt = 0.0;
    if ( exists_target_ ) {
        // Searching for the observed value of the tracking target (closest to the previous tracking position)
        search_pt = previous_target_;
        const double coast_dt = ( no_exists_time_ >= 0.0 ) ? ( now - no_exists_time_ ) : 0.0;
        leg_min_distance = leg_tracking_range_ + reacquire_radius_rate_ * coast_dt;
    } else if ( target_lost_time_ >= 0.0 && ( now - target_lost_time_ ) <= reacquire_tolerance_ ) {
        // Re-acquire the same person after an occlusion, gate grows with time.
        reacquire_mode = true;
        lost_dt = now - target_lost_time_;
        // Method C: predict the emergence point by projecting the pre-occlusion motion
        // forward (capped by reacquire_lead_max_). The person is expected to keep heading
        // the way they were going and come out the far side of the obstacle, so we search
        // there rather than exactly where they vanished.
        double lead_x = last_target_vel_.x * lost_dt;
        double lead_y = last_target_vel_.y * lost_dt;
        const double lead = std::hypotf( lead_x, lead_y );
        if ( lead > reacquire_lead_max_ && lead > 1e-6 ) {
            lead_x *= reacquire_lead_max_ / lead;
            lead_y *= reacquire_lead_max_ / lead;
        }
        search_pt.x = last_target_pos_.x + lead_x;
        search_pt.y = last_target_pos_.y + lead_y;
        leg_min_distance = leg_tracking_range_ + reacquire_radius_rate_ * lost_dt;
    } else {
        // Determine targets to track (search around the operator-seeded point, or the
        // robot origin by default -- see cold_acquire_seed_).
        search_pt = cold_acquire_seed_;
        leg_min_distance = target_range_;
    }
    // Never let the gate exceed the cold-acquisition range (avoid grabbing a far person).
    if ( leg_min_distance > target_range_ ) leg_min_distance = target_range_;

    // Furniture-leg / stray-detection mitigation: a cold-acquire (no target, and not
    // within the re-acquire-same-person window) never spontaneously locks onto whatever
    // leg happens to be nearest -- it only runs for manual_seed_timeout_ seconds after
    // an operator explicitly calls ~/reset_tracking or ~/select_target. Outside that
    // window, no candidate can pass the gate (distance is always >= 0).
    const bool cold_acquire = ( !exists_target_ && !reacquire_mode );
    const bool manual_seed_window_open =
        ( manual_seed_time_ >= 0.0 ) && ( ( now - manual_seed_time_ ) <= manual_seed_timeout_ );
    if ( cold_acquire && require_explicit_target_selection_ && !manual_seed_window_open ) {
        leg_min_distance = -1.0;
    }

    bool exists_leg_pt = false, exists_body_pt = false;
    int result;
    double best_cost = std::numeric_limits<double>::max();
    for ( const auto& pose : leg_poses ) {

        const double distance = std::hypotf( pose.position.x - search_pt.x, pose.position.y - search_pt.y );
        if ( distance >= leg_min_distance ) continue; // outside the search gate
        double cost = distance;
        if ( reacquire_mode ) {
            // Method D: weak re-ID by motion continuity. A leg that reappears as the SAME
            // person should imply a velocity (from the last known position over the elapsed
            // time) close to the pre-occlusion velocity. Penalise mismatching candidates so
            // we prefer the consistent one over a merely-nearest stranger.
            const double denom = std::max( lost_dt, 1e-2 );
            const double impl_vx = ( pose.position.x - last_target_pos_.x ) / denom;
            const double impl_vy = ( pose.position.y - last_target_pos_.y ) / denom;
            cost += reacquire_vel_weight_ * std::hypotf( impl_vx - last_target_vel_.x, impl_vy - last_target_vel_.y );
        }
        if ( cost < best_cost ) {
            best_cost = cost;
            leg_pt = pose.position;
            exists_leg_pt = true;
        }
    }

    double min_distance = ( exists_target_ ) ? body_tracking_range_ : target_range_;
    if ( cold_acquire && require_explicit_target_selection_ && !manual_seed_window_open ) {
        min_distance = -1.0;
    }
    for ( const auto& detection : body_poses ) {
        
        if (detection.results.empty()) continue;
        
        std::string class_id = detection.results[0].hypothesis.class_id;
        // In the MS COCO dataset, people are output as "0" or "person"
        if (class_id != "0" && class_id != "person") {
            continue; // If it is not a person, it will be ignored and the distance calculation below will not be performed.
        }

        double distance = std::hypotf( detection.bbox.center.position.x - search_pt.x, detection.bbox.center.position.y - search_pt.y );

        if ( min_distance > distance ) {
            min_distance = distance;
            body_pt = detection.bbox.center.position;
            exists_body_pt = true;
        }
    }
    Eigen::Vector2f leg_observed( leg_pt.x, leg_pt.y );
    Eigen::Vector2f body_observed( body_pt.x, body_pt.y );
    if ( !exists_leg_pt && !exists_body_pt ) result = Status::NO_EXISTS;
    else if ( exists_leg_pt && !exists_body_pt ) result = Status::EXISTS_LEG;
    else if ( !exists_leg_pt && exists_body_pt ) result = Status::EXISTS_BODY;
    else result = Status::EXISTS_LEG_AND_BODY;
    *leg_observed_value = leg_observed;
    *body_observed_value = body_observed;
    return result;
}

bool multiple_sensor_person_tracking::PersonTracker::searchObstacles( const geometry_msgs::msg::Point& search_pt,  const PointCloud::Ptr input_cloud, sensor_msgs::msg::PointCloud2* obstacles ) {

    // Build a colored copy of the scan cloud:
    //   - every point is WHITE by default
    //   - points belonging to the tracked leg (within target_cloud_radius_ of
    //     'search_pt') are painted GREEN
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud( new pcl::PointCloud<pcl::PointXYZRGB>() );
    colored_cloud->points.reserve( input_cloud->points.size() );
    for ( const auto& p : input_cloud->points ) {
        pcl::PointXYZRGB cp;
        cp.x = p.x; cp.y = p.y; cp.z = p.z;
        cp.r = 255; cp.g = 255; cp.b = 255; // default: white
        colored_cloud->points.push_back( cp );
    }
    colored_cloud->width  = static_cast<uint32_t>( colored_cloud->points.size() );
    colored_cloud->height = 1;
    colored_cloud->is_dense = input_cloud->is_dense;

    // Paint the detected leg green (skip if the cloud is empty or the target is invalid)
    if ( !input_cloud->points.empty() &&
         std::isfinite(search_pt.x) && std::isfinite(search_pt.y) ) {
        PointT p_q;
        p_q.x = search_pt.x;
        p_q.y = search_pt.y;
        p_q.z = input_cloud->points[0].z;
        std::vector<int> k_indices;
        std::vector<float> k_sqr_distances;
        flann_.setInputCloud( input_cloud );
        int num_match = flann_.radiusSearch( p_q, target_cloud_radius_, k_indices, k_sqr_distances, 0 );
        for ( int i = 0; i < num_match; ++i ) {
            auto& cp = colored_cloud->points[ k_indices[i] ];
            cp.r = 0; cp.g = 255; cp.b = 0; // leg: green
        }
    }

    pcl::toROSMsg( *colored_cloud, *obstacles );
    obstacles->header.frame_id = input_cloud->header.frame_id;
    obstacles->header.stamp = this->get_clock()->now();

    return true;
}

geometry_msgs::msg::PointStamped multiple_sensor_person_tracking::PersonTracker::transformPoint (
    const std::string& org_frame,
    const std::string& target_frame,
    const geometry_msgs::msg::Point& point)
{
    geometry_msgs::msg::PointStamped pt_transformed;
    geometry_msgs::msg::PointStamped pt;
    pt.header.frame_id = org_frame;
    // Use latest available TF to avoid tiny "future extrapolation" races
    // between incoming sensor timestamps and tf publication timing.
    pt.header.stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    pt.point = point;
    try{
        tfBuffer_.transform(pt, pt_transformed, target_frame);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
    return pt_transformed;
}

void multiple_sensor_person_tracking::PersonTracker::compensateEgoMotion() {
    // Every piece of tracking state -- the Kalman filter's position and velocity,
    // previous_target_, the position/velocity recorded at loss (Methods A/C/D) and the
    // operator-seeded cold-acquire point -- is expressed in target_frame_, which is
    // bolted to the robot. Between two detections the robot moves, so those stored
    // values refer to a frame that no longer exists: to the tracker, a perfectly
    // stationary person appears to sweep sideways at omega * distance.
    //
    // That apparent motion is what makes turns lose the target. DR-SPAAM runs at
    // ~2 Hz on CPU, so one frame is ~0.5 s; a 0.8 rad/s pivot (Nav2's
    // rotate_to_heading_angular_vel) displaces a person 2 m away by 0.8 m in a single
    // frame, which on its own consumes most of the leg_tracking_range gate. Worse, the
    // filter absorbs that sweep as the PERSON's velocity, so the prediction keeps
    // swinging after the robot stops turning and the recorded velocity misdirects the
    // re-acquisition search.
    //
    // Re-expressing the state in the current target_frame_ separates "the robot moved"
    // from "the person moved", so the gate is spent on the latter only. Widening the
    // gate instead is not an option: it brings back the furniture-leg mis-association
    // seen on the real robot (VISION.md section 4).
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
        // odom <- target_frame (current). Latest-available TF, consistent with the rest
        // of this node; the delta only needs to be self-consistent between frames.
        tf_msg = tfBuffer_.lookupTransform(
            odom_frame_name_, target_frame_, tf2::TimePointZero );
    } catch ( const tf2::TransformException& ex ) {
        // No odom yet (startup) or the chain broke. Drop the reference so the next
        // frame starts fresh -- never apply a delta measured across an interval we
        // did not actually observe.
        RCLCPP_WARN_THROTTLE( this->get_logger(), *this->get_clock(), 2000,
            "Ego-motion compensation skipped ('%s' -> '%s' unavailable): %s",
            odom_frame_name_.c_str(), target_frame_.c_str(), ex.what() );
        has_previous_tracking_pose_ = false;
        return;
    }

    tf2::Transform current_pose;
    tf2::fromMsg( tf_msg.transform, current_pose );

    if ( has_previous_tracking_pose_ ) {
        // current_frame <- previous_frame
        const tf2::Transform delta = current_pose.inverse() * previous_tracking_pose_;
        const tf2::Vector3 delta_translation = delta.getOrigin();
        const double delta_yaw = tf2::getYaw( delta.getRotation() );

        // Guard against an odom discontinuity (a reset, or a garbage TF) throwing the
        // tracker somewhere it can never recover from. Real motion between detections
        // is far below these bounds: the platform tops out around 0.3 m/s and 1 rad/s.
        const double translation_norm =
            std::hypot( delta_translation.x(), delta_translation.y() );
        if ( translation_norm > 2.0 || std::fabs( delta_yaw ) > 2.0 ) {
            RCLCPP_WARN( this->get_logger(),
                "Ego-motion compensation skipped: implausible odom jump "
                "(%.2f m, %.2f rad between detections)", translation_norm, delta_yaw );
            previous_tracking_pose_ = current_pose;
            return;
        }

        const double cos_yaw = std::cos( delta_yaw );
        const double sin_yaw = std::sin( delta_yaw );

        // Points move with the full rigid transform; velocities only rotate.
        auto move_point = [&]( geometry_msgs::msg::Point& p ) {
            const double x = p.x, y = p.y;
            p.x = cos_yaw * x - sin_yaw * y + delta_translation.x();
            p.y = sin_yaw * x + cos_yaw * y + delta_translation.y();
        };
        auto rotate_vector = [&]( geometry_msgs::msg::Point& v ) {
            const double x = v.x, y = v.y;
            v.x = cos_yaw * x - sin_yaw * y;
            v.y = sin_yaw * x + cos_yaw * y;
        };

        move_point( previous_target_ );
        move_point( last_target_pos_ );
        // The operator picked a point in the world, not a bearing off the robot, so it
        // has to follow the world too while the manual_seed_timeout_ window is open.
        move_point( cold_acquire_seed_ );
        rotate_vector( last_target_vel_ );
        rotate_vector( previous_vel_ );

        if ( kf_ ) {
            Eigen::Matrix2f rotation;
            rotation << static_cast<float>( cos_yaw ), static_cast<float>( -sin_yaw ),
                        static_cast<float>( sin_yaw ), static_cast<float>(  cos_yaw );
            const Eigen::Vector2f translation(
                static_cast<float>( delta_translation.x() ),
                static_cast<float>( delta_translation.y() ) );
            kf_->applyFrameTransform( rotation, translation );
        }
    }

    previous_tracking_pose_ = current_pose;
    has_previous_tracking_pose_ = true;
}

void multiple_sensor_person_tracking::PersonTracker::publishFollowing() {
    // Derive the stop signal for the real robot:
    //   - detection interrupted this frame (status == NO_EXISTS), or
    //   - the target was (possibly) switched to a different person (latched).
    std_msgs::msg::Bool stop_msg;
    const bool detection_lost = ( following_position_->status == Status::NO_EXISTS );
    stop_msg.data = detection_lost || target_changed_latched_;

    pub_following_position_->publish( *following_position_ );
    if ( pub_stop_ ) pub_stop_->publish( stop_msg );

    // Log only on transitions so the terminal clearly shows when to stop / resume.
    if ( stop_msg.data && !prev_stop_ ) {
        if ( target_changed_latched_ )
            RCLCPP_ERROR( this->get_logger(),
                "\033[1;41m STOP \033[m target CHANGED to a different person -> stop the robot "
                "(call the ~/reset_tracking service to resume)" );
        else
            RCLCPP_WARN( this->get_logger(),
                "\033[1;43m\033[30m STOP \033[m target detection LOST -> stop the robot" );
    } else if ( !stop_msg.data && prev_stop_ ) {
        RCLCPP_INFO( this->get_logger(),
            "\033[1;42m\033[30m RESUME \033[m target tracking recovered" );
    }
    prev_stop_ = stop_msg.data;
}

void multiple_sensor_person_tracking::PersonTracker::resetTrackingCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response )
{
    // Clear the safety latch and drop the current lock so the next frame re-acquires
    // the nearest person (operator positions the correct person in front of the robot).
    target_changed_latched_ = false;
    exists_target_ = false;
    target_lost_time_ = -1.0;
    no_exists_time_ = -1.0;
    // Reproduce the original "grab nearest leg to robot origin" behavior even when
    // require_explicit_target_selection_ blocks spontaneous cold-acquire: seed at the
    // origin and open a short manual-acquire window.
    cold_acquire_seed_ = geometry_msgs::msg::Point();
    manual_seed_time_ = this->get_clock()->now().seconds();
    RCLCPP_INFO( this->get_logger(), "reset_tracking: safety latch cleared, re-acquiring target." );
    response->success = true;
    response->message = "tracking reset";
}

void multiple_sensor_person_tracking::PersonTracker::selectTargetCallback(
    const std::shared_ptr<multiple_sensor_person_tracking::srv::SelectTarget::Request> request,
    std::shared_ptr<multiple_sensor_person_tracking::srv::SelectTarget::Response> response )
{
    geometry_msgs::msg::Point seed;
    if ( request->candidate_index >= 0 ) {
        const auto idx = static_cast<size_t>( request->candidate_index );
        if ( idx >= last_candidates_.positions.size() ) {
            response->success = false;
            response->message = "candidate_index out of range for the last published candidate list";
            return;
        }
        seed = last_candidates_.positions[idx];
    } else {
        if ( !std::isfinite(request->x) || !std::isfinite(request->y) ) {
            response->success = false;
            response->message = "x/y must be finite when candidate_index < 0";
            return;
        }
        seed.x = request->x;
        seed.y = request->y;
    }

    // Same mechanism ~/reset_tracking uses (cold-acquire gated nearest-neighbor), just
    // seeded at an operator-chosen point instead of the robot origin.
    cold_acquire_seed_ = seed;
    manual_seed_time_ = this->get_clock()->now().seconds();
    target_changed_latched_ = false;
    exists_target_ = false;
    target_lost_time_ = -1.0;
    no_exists_time_ = -1.0;

    RCLCPP_INFO( this->get_logger(),
        "select_target: seeding target at (%.2f, %.2f); awaiting association.",
        seed.x, seed.y );
    response->success = true;
    response->message = "target seeded";
}

void multiple_sensor_person_tracking::PersonTracker::scan_callback (const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg)
{
    if (!active_) {
        return;
    }
    scan_msg_ = scan_msg;
}

void multiple_sensor_person_tracking::PersonTracker::nontravelableRegionCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& nontravelable_region_msg)
{
    if (!active_) {
        return;
    }
    if (nontravelable_region_msg->header.frame_id.empty()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Skipping non-travelable region cloud with empty frame_id.");
        return;
    }

    PointCloud temp_cloud;
    try {
        pcl::fromROSMsg(*nontravelable_region_msg, temp_cloud);
    } catch (std::runtime_error &ex) {
        RCLCPP_ERROR(this->get_logger(), "nontravelableRegionCallback() conversion failed: %s", ex.what());
        return;
    }

    // Transform to the same frame as /obstacles
    sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tfBuffer_.lookupTransform(
            target_frame_,
            nontravelable_region_msg->header.frame_id,
            tf2::TimePointZero);

        tf2::doTransform(*nontravelable_region_msg, transformed_cloud_msg, transform);

        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl::fromROSMsg(transformed_cloud_msg, transformed_cloud);

        *cloud_nontravelable_region_ = transformed_cloud;
        cloud_nontravelable_region_->header.frame_id = target_frame_; 
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform non-travelable region cloud: %s", ex.what());
        return;
    }
}

void multiple_sensor_person_tracking::PersonTracker::dr_spaam_callback(const geometry_msgs::msg::PoseArray::ConstSharedPtr &dr_spaam_msg) 
{
    if (!active_) {
        return;
    }
    dr_spaam_msg_ = dr_spaam_msg;
    if (detection_mode_ == DetectionMode::LEG) {
        auto empty_body_msg = std::make_shared<vision_msgs::msg::Detection3DArray>();
        empty_body_msg->header.stamp = dr_spaam_msg->header.stamp;
        empty_body_msg->header.frame_id = target_frame_;
        callbackPoseArray(empty_body_msg);
    }
}

void multiple_sensor_person_tracking::PersonTracker::callbackPoseArray ( const vision_msgs::msg::Detection3DArray::ConstSharedPtr &body_msg ) {
    if (!active_) {
        return;
    }
    
    std::cout << "\n====================================" << std::endl;
    // variable initialization
    std::string target_frame = target_frame_;
    sensor_msgs::msg::PointCloud2 cloud_scan_msg;
    Eigen::Vector4f estimated_value( 0.0, 0.0, 0.0, 0.0 );

    rclcpp::Time current_time = this->get_clock()->now();
    double dt = (current_time - previous_time_).seconds();
    previous_time_ = current_time;

    // Bring the tracking state into the current robot frame before anything reads it.
    // Done up front and unconditionally so that the stored reference pose and the state
    // always advance together -- an early return below must not leave the state
    // expressed in one frame and the reference in another.
    compensateEgoMotion();

    const bool use_leg_detection = detection_mode_ != DetectionMode::BODY;
    const bool use_body_detection = detection_mode_ != DetectionMode::LEG;

    // Wait until a valid scan frame is available.
    if (!scan_msg_ || scan_msg_->header.frame_id.empty()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Waiting for a valid LaserScan frame on scan_topic_name.");
        return;
    }

    // Sensor data to TF2 conversion
    try {
        sensor_msgs::msg::LaserScan scan_msg = *scan_msg_;
        if (!scan_frame_name_.empty()) {
            scan_msg.header.frame_id = scan_frame_name_;
        }
        if (!tfBuffer_.canTransform(
                target_frame,
                scan_msg.header.frame_id,
                scan_msg.header.stamp,
                tf2::durationFromSec(0.05))) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for transform from '%s' to '%s'.",
                scan_msg.header.frame_id.c_str(),
                target_frame.c_str());
            return;
        }
        projector_.transformLaserScanToPointCloud( target_frame, scan_msg, cloud_scan_msg, tfBuffer_ );
        pcl::fromROSMsg<PointT>( cloud_scan_msg, *cloud_scan_);
        cloud_scan_->header.frame_id = target_frame;
    } catch ( const tf2::TransformException& ex ) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        following_position_->pose.position.x = 0.0;
        following_position_->pose.position.y = 0.0;
        following_position_->rotation_position = following_position_->pose.position;
        following_position_->status = Status::NO_EXISTS;
        publishFollowing();
        return;
    }

    std::vector<geometry_msgs::msg::Pose> leg_detections_in_target = dr_spaam_msg_->poses;
    std::string leg_array_frame = dr_spaam_msg_->header.frame_id;
    if (!scan_frame_name_.empty() && scan_msg_ && leg_array_frame == scan_msg_->header.frame_id) {
        leg_array_frame = scan_frame_name_;
    }
    if (use_leg_detection && !leg_array_frame.empty() && leg_array_frame != target_frame_) {
        for (auto & pose : leg_detections_in_target) {
            const auto transformed = transformPoint(leg_array_frame, target_frame_, pose.position);
            pose.position = transformed.point;
        }
    }

    // Publish the live candidate list (already transformed to target_frame_) so an
    // operator can pick a specific person via ~/select_target, independent of whether
    // the tracker currently has a locked target. Placed before any early return below
    // so the list keeps updating even while require_explicit_target_selection_ blocks
    // spontaneous cold-acquire.
    if ( pub_candidates_ ) {
        multiple_sensor_person_tracking::msg::PersonCandidates candidates_msg;
        candidates_msg.header.stamp = current_time;
        candidates_msg.header.frame_id = target_frame_;
        candidates_msg.positions.reserve( leg_detections_in_target.size() );
        for ( const auto & pose : leg_detections_in_target ) {
            candidates_msg.positions.push_back( pose.position );
        }
        last_candidates_ = candidates_msg;
        pub_candidates_->publish( candidates_msg );
    }

    if ( !exists_target_ && use_body_detection && body_msg->detections.size() == 0) {
        if ( !use_leg_detection || leg_detections_in_target.size() == 0 ) {
            RCLCPP_ERROR(this->get_logger(), "Result :          NO_EXISTS (DR-SPAAM)" );
            exists_target_ = false;
            following_position_->pose.position.x = 0.0;
            following_position_->pose.position.y = 0.0;
            following_position_->rotation_position = following_position_->pose.position;
            following_position_->status = Status::NO_EXISTS;
            publishFollowing();
            return;
        }
        if( attention_leg_time_ == -1.0 ) attention_leg_time_ = this->get_clock()->now().seconds();
        exists_target_ = false;
        std::vector<geometry_msgs::msg::Pose> leg_poses = leg_detections_in_target;
        // Rotate the RGB-D sensor in the direction in which the leg_poses
        // Sort by proximity
        std::sort(
            leg_poses.begin(),
            leg_poses.end(),
            []( const auto & a, const auto & b)
            { return std::hypotf(a.position.x, a.position.y) < std::hypotf(b.position.x, b.position.y); } );
        // set the position of attention_leg_idx_ -> (if attention_leg_idx_ is larger than the array, modify)
        // change attention_leg_idx_ in 2 seconds
        if ( this->get_clock()->now().seconds() - attention_leg_time_ >= 2.0 ) {
            attention_leg_idx_ = (attention_leg_idx_ + 1) % leg_poses.size();
            attention_leg_time_ = this->get_clock()->now().seconds();
        } else if (attention_leg_idx_ >= leg_poses.size()) {
            attention_leg_idx_ = leg_poses.size() - 1;
        }
        following_position_->rotation_position.x = leg_poses[attention_leg_idx_].position.x;
        following_position_->rotation_position.y = leg_poses[attention_leg_idx_].position.y;
        following_position_->pose.position.x = 0.0;
        following_position_->pose.position.y = 0.0;
        following_position_->status = Status::NO_EXISTS;
        publishFollowing();
        RCLCPP_ERROR(this->get_logger(), "Result :          NO_EXISTS (Object) attention_leg_idx = %d",attention_leg_idx_ );
        return;
    } else {
        attention_leg_time_ = -1.0;
        attention_leg_idx_ = 0;
    }
    // Transform body detections into tracker target frame so body/leg fusion
    // is computed in one coordinate system.
    std::vector<vision_msgs::msg::Detection3D> body_detections_in_target = body_msg->detections;
    const std::string array_frame = body_msg->header.frame_id;
    for (auto & detection : body_detections_in_target) {
        std::string src_frame = detection.header.frame_id;
        if (src_frame.empty()) {
            src_frame = array_frame;
        }
        if (!src_frame.empty() && src_frame != target_frame_) {
            const auto transformed = transformPoint(src_frame, target_frame_, detection.bbox.center.position);
            detection.bbox.center.position = transformed.point;
            detection.header.frame_id = target_frame_;
        }
    }

    // Searching for observables to input to the Kalman filter
    Eigen::Vector2f leg_observed_value, body_observed_value;
    const std::vector<geometry_msgs::msg::Pose> leg_observations =
        use_leg_detection ? leg_detections_in_target : std::vector<geometry_msgs::msg::Pose>{};
    const std::vector<vision_msgs::msg::Detection3D> body_observations =
        use_body_detection ? body_detections_in_target : std::vector<vision_msgs::msg::Detection3D>{};
    int result = findTwoObservationValue( leg_observations, body_observations, &leg_observed_value, &body_observed_value );
    if ( result == Status::NO_EXISTS ) {
        if ( no_exists_time_ == -1.0 ) no_exists_time_ = this->get_clock()->now().seconds();
        else {
            // Method B: coast (keep predicting) longer for a slow / stopped target, which
            // is likely dwelling behind the obstacle, and shorter for a fast one, which
            // passes through and re-emerges quickly. The effective tolerance decays from
            // coast_tolerance_max_ (at rest) toward target_change_tolerance_ as speed rises.
            const double speed = following_position_->velocity; // last tracked speed [m/s]
            const double scale = std::max( coast_speed_scale_, 1e-3 );
            double eff_tol = target_change_tolerance_
                + ( coast_tolerance_max_ - target_change_tolerance_ ) / ( 1.0 + speed / scale );
            if ( eff_tol < target_change_tolerance_ ) eff_tol = target_change_tolerance_;
            if ( this->get_clock()->now().seconds() - no_exists_time_ >= eff_tol ) {
                exists_target_ = false;
                // Method A: remember where/when we lost the target so we can re-acquire the
                // SAME person when they emerge from behind the obstacle. Method C/D reuse the
                // pre-occlusion velocity to predict the emergence point and disambiguate.
                last_target_pos_ = previous_target_;
                last_target_vel_ = previous_vel_;
                target_lost_time_ = this->get_clock()->now().seconds();
                following_position_->pose.position.x = 0.0;
                following_position_->pose.position.y = 0.0;
                following_position_->status = Status::NO_EXISTS;
                publishFollowing();
                return;
            }
        }
    } else no_exists_time_ = -1.0;

    // Tracking by Kalman Filter
    if ( !exists_target_ ) {
        // Classify this acquisition. A lock obtained after a PREVIOUS target was lost and
        // the re-acquire window already expired is treated as a (possibly) different person
        // -> raise the safety latch. A fresh startup lock (no prior target) and an in-window
        // re-acquire (Methods A-D recovered the same person) are NOT switches.
        const double t_now = current_time.seconds();
        const bool prior_loss = ( target_lost_time_ >= 0.0 );
        const bool within_reacquire = prior_loss && ( t_now - target_lost_time_ ) <= reacquire_tolerance_;
        const bool possible_switch = prior_loss && !within_reacquire;
        if ( result == Status::EXISTS_BODY || result == Status::EXISTS_LEG_AND_BODY ) {
            kf_->init( body_observed_value );
            estimated_value[0] = body_observed_value[0];
            estimated_value[1] = body_observed_value[1];
            following_position_->rotation_position.x = body_observed_value[0];
            following_position_->rotation_position.y = body_observed_value[1];
            exists_target_ = true;
            if ( possible_switch ) target_changed_latched_ = true; // different person -> latch STOP
            target_lost_time_ = -1.0; // (re)acquired -> stop re-acquire mode
        } else if ( result == Status::EXISTS_LEG ) {
            kf_->init( leg_observed_value );
            estimated_value[0] = leg_observed_value[0];
            estimated_value[1] = leg_observed_value[1];
            following_position_->rotation_position.x = leg_observed_value[0];
            following_position_->rotation_position.y = leg_observed_value[1];
            exists_target_ = true;
            if ( possible_switch ) target_changed_latched_ = true; // different person -> latch STOP
            target_lost_time_ = -1.0; // (re)acquired -> stop re-acquire mode
        } else {
            RCLCPP_ERROR(this->get_logger(), "Result :          NO_EXISTS" );
            exists_target_ = false;
            following_position_->pose.position.x = 0.0;
            following_position_->pose.position.y = 0.0;
            following_position_->status = Status::NO_EXISTS;
            publishFollowing();
            return;
        }
    } else {
        if( result == Status::EXISTS_LEG ) {
            kf_->compute( dt, leg_observed_value, &estimated_value );
            following_position_->rotation_position.x = estimated_value[0];
            following_position_->rotation_position.y = estimated_value[1];
        } else if( result == Status::EXISTS_BODY ) {
            kf_->compute( dt, body_observed_value, &estimated_value );
            following_position_->rotation_position.x = body_observed_value[0];
            following_position_->rotation_position.y = body_observed_value[1];
        } else if ( result == Status::EXISTS_LEG_AND_BODY ) {
            kf_->compute( dt, leg_observed_value, body_observed_value, &estimated_value );
            following_position_->rotation_position.x = body_observed_value[0];
            following_position_->rotation_position.y = body_observed_value[1];
        } else {
            kf_->compute( dt, &estimated_value );
            following_position_->rotation_position.x = estimated_value[0];
            following_position_->rotation_position.y = estimated_value[1];
        }
    }

    // following_position_ : pose :
    following_position_->pose.position.x = estimated_value[0];
    following_position_->pose.position.y = estimated_value[1];
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0, 0, std::atan2(estimated_value[3], estimated_value[2]));
    geometry_msgs::msg::Quaternion quat_msg;
    tf2::convert(quat_tf, quat_msg);

    following_position_->pose.orientation = quat_msg;
    following_position_->velocity = std::hypotf(estimated_value[2], estimated_value[3]);
    following_position_->status = result;

    // search obstacles :
    sensor_msgs::msg::PointCloud2 obstacles;
    outrem_.setInputCloud( cloud_scan_ );
    outrem_.filter ( *cloud_scan_ );
    voxel_.setInputCloud( cloud_scan_ );
    voxel_.filter ( *cloud_scan_ );
    bool can_pub_obstacles = searchObstacles( following_position_->pose.position, cloud_scan_, &obstacles );

    // following_position_ : header :
    if ( can_pub_obstacles ){
        pub_obstacles_->publish( obstacles );
        following_position_->header.stamp = this->get_clock()->now();
        publishFollowing();
        pub_target_odom_->publish( transformPoint( target_frame_, odom_frame_name_, following_position_->pose.position ) );
    } 

    if ( display_marker_ ) {
        marker_array_->markers.clear();
        marker_array_->markers.push_back( makeLegPoseMarker(leg_detections_in_target) );
        marker_array_->markers.push_back( makeLegAreaMarker(leg_detections_in_target) );
        marker_array_->markers.push_back( makeBodyPoseMarker(body_detections_in_target) );
        marker_array_->markers.push_back( makeTargetPoseMarker(estimated_value) );
        marker_array_->markers.push_back( makeTargetTextMarker(estimated_value) );
        pub_marker_->publish ( *marker_array_ );
    }
    previous_target_ = following_position_->pose.position;
    // Keep the latest estimated velocity so it can seed the emergence prediction (C/D) at loss.
    previous_vel_.x = estimated_value[2];
    previous_vel_.y = estimated_value[3];

    RCLCPP_INFO( this->get_logger(), "\033[1mResult\033[m = %s",
        ( following_position_->status == Status::EXISTS_LEG ? "\033[1;36m EXISTS_LEG \033[m" :
        ( following_position_->status == Status::EXISTS_BODY ? "\033[1;33m EXISTS_BODY \033[m" :
        ( following_position_->status == Status::EXISTS_LEG_AND_BODY ? "\033[1;32m EXISTS_LEG_AND_BODY \033[m" : "\033[1;31m NO_EXISTS \033[m")) ));
    RCLCPP_INFO( this->get_logger(), "\033[1mTarget\033[m = %5.3f [m]\t%5.3f [m]", following_position_->pose.position.x, following_position_->pose.position.y );

    return;
}

multiple_sensor_person_tracking::CallbackReturn multiple_sensor_person_tracking::PersonTracker::on_configure(const rclcpp_lifecycle::State &) {

    // Declare parameters
    try {
        this->declare_parameter<std::string>("scan_topic_name", "/scan");
        this->declare_parameter<std::string>("pointcloud_nontravelable_region_topic_name", "sobits_follower/multiple_sensor_person_tracking/pointcloud_nontravelable_region");
        this->declare_parameter<std::string>("dr_spaam_topic_name", "/dr_spaam_detections");
        this->declare_parameter<std::string>("body_detection_topic_name", "/sobits_follower/object_3d_poses");
        this->declare_parameter<std::string>("target_frame", "base_footprint");
        this->declare_parameter<std::string>("odom_frame_name", "odom");
        this->declare_parameter<std::string>("scan_frame_name", "");
        this->declare_parameter<std::string>("detection_mode", "body_leg");
        this->declare_parameter<bool>("merge_nontravelable_region", false);
        this->declare_parameter<double>("leg_tracking_range", 2.5);
        this->declare_parameter<double>("body_tracking_range", 2.5);
        this->declare_parameter<double>("outlier_radius", 0.1);
        this->declare_parameter<int>("outlier_min_pts", 2);
        this->declare_parameter<double>("leaf_size", 0.1);
        this->declare_parameter<double>("target_cloud_radius", 0.4);
        this->declare_parameter<double>("target_change_tolerance", 2.0);
        this->declare_parameter<double>("reacquire_tolerance", 3.0);
        this->declare_parameter<double>("reacquire_radius_rate", 0.5);
        this->declare_parameter<double>("reacquire_lead_max", 2.0);
        this->declare_parameter<double>("reacquire_vel_weight", 1.0);
        this->declare_parameter<double>("coast_tolerance_max", 5.0);
        this->declare_parameter<double>("coast_speed_scale", 0.5);
        this->declare_parameter<bool>("display_marker", true);
        this->declare_parameter<double>("target_range", 3.0);
        this->declare_parameter<bool>("require_explicit_target_selection", true);
        this->declare_parameter<double>("manual_seed_timeout", 5.0);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    }

    // Retrieve parameter values
    auto scan_topic_name = this->get_parameter("scan_topic_name").as_string();
    auto pointcloud_nontravelable_region_topic_name = this->get_parameter("pointcloud_nontravelable_region_topic_name").as_string();
    auto dr_spaam_topic_name = this->get_parameter("dr_spaam_topic_name").as_string();
    auto body_detection_topic_name = this->get_parameter("body_detection_topic_name").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    odom_frame_name_ = this->get_parameter("odom_frame_name").as_string();
    scan_frame_name_ = this->get_parameter("scan_frame_name").as_string();
    auto detection_mode = this->get_parameter("detection_mode").as_string();
    merge_nontravelable_region_ = this->get_parameter("merge_nontravelable_region").as_bool();
    leg_tracking_range_ = this->get_parameter("leg_tracking_range").as_double();
    body_tracking_range_ = this->get_parameter("body_tracking_range").as_double();
    target_change_tolerance_ = this->get_parameter("target_change_tolerance").as_double();
    reacquire_tolerance_ = this->get_parameter("reacquire_tolerance").as_double();
    reacquire_radius_rate_ = this->get_parameter("reacquire_radius_rate").as_double();
    reacquire_lead_max_ = this->get_parameter("reacquire_lead_max").as_double();
    reacquire_vel_weight_ = this->get_parameter("reacquire_vel_weight").as_double();
    coast_tolerance_max_ = this->get_parameter("coast_tolerance_max").as_double();
    coast_speed_scale_ = this->get_parameter("coast_speed_scale").as_double();
    display_marker_ = this->get_parameter("display_marker").as_bool();
    target_range_ = this->get_parameter("target_range").as_double();
    require_explicit_target_selection_ = this->get_parameter("require_explicit_target_selection").as_bool();
    manual_seed_timeout_ = this->get_parameter("manual_seed_timeout").as_double();
    if (detection_mode == "leg") {
        detection_mode_ = DetectionMode::LEG;
    } else if (detection_mode == "body") {
        detection_mode_ = DetectionMode::BODY;
    } else if (detection_mode == "body_leg") {
        detection_mode_ = DetectionMode::BODY_LEG;
    } else {
        RCLCPP_WARN(
            this->get_logger(),
            "Unknown detection_mode '%s'. Falling back to 'body_leg'.",
            detection_mode.c_str());
        detection_mode_ = DetectionMode::BODY_LEG;
        detection_mode = "body_leg";
    }
    RCLCPP_INFO(this->get_logger(), "detection_mode: %s", detection_mode.c_str());

    // Initialize class members
    tf_sub_.reset(new tf2_ros::TransformListener(tfBuffer_));
    cloud_scan_.reset(new PointCloud());
    marker_array_.reset(new visualization_msgs::msg::MarkerArray);
    following_position_.reset( new multiple_sensor_person_tracking::msg::FollowingPosition );
    scan_msg_.reset( new sensor_msgs::msg::LaserScan );
    cloud_nontravelable_region_.reset( new PointCloud() );
    dr_spaam_msg_.reset( new geometry_msgs::msg::PoseArray() );

    // Create subscribers with sensor QoS to interoperate with Gazebo/bridge topics.
    auto sensor_qos = rclcpp::SensorDataQoS();
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_name, sensor_qos, std::bind(&PersonTracker::scan_callback, this, std::placeholders::_1));

    sub_nontravelable_region_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_nontravelable_region_topic_name, sensor_qos, std::bind(&PersonTracker::nontravelableRegionCallback, this, std::placeholders::_1));

    sub_dr_spaam_ = create_subscription<geometry_msgs::msg::PoseArray>(
        dr_spaam_topic_name, sensor_qos, std::bind(&PersonTracker::dr_spaam_callback, this, std::placeholders::_1));
    
    sub_image_ = create_subscription<vision_msgs::msg::Detection3DArray>(
        body_detection_topic_name, sensor_qos, std::bind(&PersonTracker::callbackPoseArray, this, std::placeholders::_1));
    
    // Create publishers
    pub_following_position_ = create_publisher< multiple_sensor_person_tracking::msg::FollowingPosition >( "sobits_follower/multiple_sensor_person_tracking/following_position", 1 );
    pub_marker_ = create_publisher< visualization_msgs::msg::MarkerArray >( "sobits_follower/multiple_sensor_person_tracking/tracker_marker", 1 );
    pub_obstacles_ = create_publisher< sensor_msgs::msg::PointCloud2 >( "sobits_follower/multiple_sensor_person_tracking/obstacles", 1 );
    pub_target_odom_ = create_publisher< geometry_msgs::msg::PointStamped >( "sobits_follower/multiple_sensor_person_tracking/target_postion_odom", 1 );
    // Safety stop signal: true = real robot should stop (target lost or switched).
    pub_stop_ = create_publisher< std_msgs::msg::Bool >( "sobits_follower/multiple_sensor_person_tracking/stop_following", 1 );
    // Service to clear the "target changed" latch and re-acquire the nearest person.
    srv_reset_ = create_service< std_srvs::srv::Trigger >(
        "~/reset_tracking",
        std::bind(&PersonTracker::resetTrackingCallback, this, std::placeholders::_1, std::placeholders::_2) );
    // Live candidate list, for operator target selection.
    pub_candidates_ = create_publisher< multiple_sensor_person_tracking::msg::PersonCandidates >(
        "sobits_follower/multiple_sensor_person_tracking/person_candidates", 1 );
    // Service to force-seed a specific candidate/point as the new tracked target.
    srv_select_target_ = create_service< multiple_sensor_person_tracking::srv::SelectTarget >(
        "~/select_target",
        std::bind(&PersonTracker::selectTargetCallback, this, std::placeholders::_1, std::placeholders::_2) );

    // Initialize Kalman filter
    kf_ = std::make_unique<multiple_observation_kalman_filter::KalmanFilter>(0.033, 1000, 1.0);

    // Configure pcl filters
    outrem_.setRadiusSearch(this->get_parameter("outlier_radius").as_double());
    outrem_.setMinNeighborsInRadius(this->get_parameter("outlier_min_pts").as_int());
    outrem_.setKeepOrganized(false);

    double leaf_size = this->get_parameter("leaf_size").as_double();
    voxel_.setLeafSize(leaf_size, leaf_size, 0.0);
    target_cloud_radius_ = this->get_parameter("target_cloud_radius").as_double();

    // Initialize timers and flags
    previous_time_ = this->now();;
    exists_target_ = false;
    no_exists_time_ = -1.0;
    target_lost_time_ = -1.0;
    last_target_pos_ = geometry_msgs::msg::Point();
    last_target_vel_ = geometry_msgs::msg::Point();
    previous_vel_ = geometry_msgs::msg::Point();
    attention_leg_time_ = -1.0;
    attention_leg_idx_ = 0;
    active_ = false;
    target_changed_latched_ = false;
    prev_stop_ = false;
    cold_acquire_seed_ = geometry_msgs::msg::Point();
    manual_seed_time_ = -1.0;
    previous_tracking_pose_ = tf2::Transform::getIdentity();
    has_previous_tracking_pose_ = false;

    return CallbackReturn::SUCCESS;
}

multiple_sensor_person_tracking::CallbackReturn multiple_sensor_person_tracking::PersonTracker::on_activate(const rclcpp_lifecycle::State &) {
    active_ = true;
    pub_following_position_->on_activate();
    pub_marker_->on_activate();
    pub_obstacles_->on_activate();
    pub_target_odom_->on_activate();
    pub_stop_->on_activate();
    pub_candidates_->on_activate();
    // Start un-latched so a fresh activation does not keep the robot stopped.
    target_changed_latched_ = false;
    prev_stop_ = false;
    manual_seed_time_ = -1.0;
    previous_time_ = this->now();
    // Any robot movement while deactivated must not be applied as a single jump on the
    // first frame after activation.
    has_previous_tracking_pose_ = false;
    return CallbackReturn::SUCCESS;
}

multiple_sensor_person_tracking::CallbackReturn multiple_sensor_person_tracking::PersonTracker::on_deactivate(const rclcpp_lifecycle::State &) {
    active_ = false;
    if (pub_following_position_) pub_following_position_->on_deactivate();
    if (pub_marker_) pub_marker_->on_deactivate();
    if (pub_obstacles_) pub_obstacles_->on_deactivate();
    if (pub_target_odom_) pub_target_odom_->on_deactivate();
    if (pub_stop_) pub_stop_->on_deactivate();
    if (pub_candidates_) pub_candidates_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

void multiple_sensor_person_tracking::PersonTracker::resetInterfaces() {
    sub_scan_.reset();
    sub_nontravelable_region_.reset();
    sub_dr_spaam_.reset();
    sub_image_.reset();
    pub_following_position_.reset();
    pub_marker_.reset();
    pub_obstacles_.reset();
    pub_target_odom_.reset();
    pub_stop_.reset();
    srv_reset_.reset();
    pub_candidates_.reset();
    srv_select_target_.reset();
    tf_sub_.reset();
    cloud_nontravelable_region_.reset();
    kf_.reset();
    cloud_scan_.reset();
    marker_array_.reset();
    following_position_.reset();
    scan_msg_.reset();
    dr_spaam_msg_.reset();
}

multiple_sensor_person_tracking::CallbackReturn multiple_sensor_person_tracking::PersonTracker::on_cleanup(const rclcpp_lifecycle::State &) {
    active_ = false;
    resetInterfaces();
    return CallbackReturn::SUCCESS;
}

multiple_sensor_person_tracking::CallbackReturn multiple_sensor_person_tracking::PersonTracker::on_shutdown(const rclcpp_lifecycle::State &) {
    active_ = false;
    resetInterfaces();
    return CallbackReturn::SUCCESS;
}

multiple_sensor_person_tracking::CallbackReturn multiple_sensor_person_tracking::PersonTracker::on_error(const rclcpp_lifecycle::State &) {
    active_ = false;
    resetInterfaces();
    return CallbackReturn::SUCCESS;
}

RCLCPP_COMPONENTS_REGISTER_NODE(multiple_sensor_person_tracking::PersonTracker)
