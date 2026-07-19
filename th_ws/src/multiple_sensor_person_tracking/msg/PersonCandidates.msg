# Live list of leg-detection candidates, already transformed into target_frame
# (base_link on the real robot), published every frame regardless of whether
# the tracker currently has a locked target. Used by an operator UI to pick
# a specific person via PersonTracker's ~/select_target service.
std_msgs/Header header
geometry_msgs/Point[] positions
