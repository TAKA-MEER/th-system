"""vibe.sh selftest 用の合成データ。スマホを YAW 度回して 5° 傾けて置き、2.5 Hz で揺れる上部を作る。

  synth.py OUT_DIR YAW_DEG
"""
import numpy as np, rosbag2_py, sys, os
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
out = sys.argv[1]; YAW = np.radians(float(sys.argv[2]))
def writer(p, topics):
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=p, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr','cdr'))
    for n,t in topics: w.create_topic(rosbag2_py.TopicMetadata(name=n, type=t, serialization_format='cdr'))
    return w
T0 = 1_700_000_000.0; dur = 30; fs = 100
t = np.arange(0, dur, 1/fs)
# 速度: 3s静止→2s加速→5s定速0.3→1.5s減速→3s静止→旋回4s→静止
v = np.interp(t, [0,3,5,10,11.5,14.5,30], [0,0,.3,.3,0,0,0])
w = np.where((t>16)&(t<20), .5, 0.)
a = np.gradient(v, 1/fs)
# 2.5Hz の揺れ: 加速度変化で励起・減衰 0.3s
f0=2.5; z=0.08; wn=2*np.pi*f0; x=np.zeros(len(t)); xd=0; sway=np.zeros(len(t))
for i in range(1,len(t)):
    xdd = -2*z*wn*xd - wn**2*x[i-1] - a[i]*1.0
    xd += xdd/fs; x[i] = x[i-1] + xd/fs; sway[i] = xdd  # 台車に対する上部の相対加速度
ax_true = a + sway
acc_body = np.column_stack([ax_true, 0.02*np.random.randn(len(t)), 9.80665 + 0.02*np.random.randn(len(t))])
gyr_body = np.column_stack([np.zeros(len(t)), np.gradient(x,1/fs)*2, w])
# スマホの置き方: yaw YAW と x軸まわり 5°傾き
c,s=np.cos(YAW),np.sin(YAW); Rz=np.array([[c,-s,0],[s,c,0],[0,0,1]])
r=np.radians(5); Rx=np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
Rpb = (Rz@Rx).T   # body→phone
acc_ph = acc_body@Rpb.T; gyr_ph = gyr_body@Rpb.T
os.makedirs(out, exist_ok=True)
os.makedirs(out+'/phone', exist_ok=True)
rows=[]
phone_clock_off = -12.3  # スマホの時計は PC より 12.3s 遅れ
for i,ti in enumerate(t):
    st=int((T0+ti+phone_clock_off)*1e9); recv=int((T0+ti+0.01+abs(np.random.randn())*0.03)*1e9)
    a_=acc_ph[i]/9.80665; g_=gyr_ph[i]
    rows.append((recv, f"{recv},{st},imu_link,{a_[0]!r},{a_[1]!r},{a_[2]!r},{g_[0]!r},{g_[1]!r},{g_[2]!r},0,0,0,1"))
with open(out+'/phone/conduit__imu.csv','w') as fp:
    fp.write('# topic /conduit/imu\nrecv_ns,stamp_ns,frame_id,ax,ay,az,gx,gy,gz,qx,qy,qz,qw\n')
    for _,r in sorted(rows): fp.write(r+'\n')
rw = writer(out+'/robot', [('/odom','nav_msgs/msg/Odometry'),('/cmd_vel','geometry_msgs/msg/Twist'),('/esp32/imu_data','sensor_msgs/msg/Imu')])
for i in range(0,len(t),2):  # 50Hz
    ti=t[i]; o=Odometry(); o.twist.twist.linear.x=v[i]; o.twist.twist.angular.z=w[i]
    rw.write('/odom', serialize_message(o), int((T0+ti+0.005)*1e9))
    c_=Twist(); vi=np.interp(ti+0.1,t,v); c_.linear.x=vi; c_.angular.z=np.interp(ti+0.1,t,w)
    rw.write('/cmd_vel', serialize_message(c_), int((T0+ti)*1e9))
    b=Imu(); b.linear_acceleration.x=a[i]+0.02*np.random.randn(); b.linear_acceleration.z=9.8; b.angular_velocity.z=w[i]
    rw.write('/esp32/imu_data', serialize_message(b), int((T0+ti+0.003)*1e9))
del rw
