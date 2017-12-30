import rospy
from chai_msg.msg import ObjectState

global prev_time, cur_time, ctr
prev_time = 0.0
cur_time = 0.0
ctr = 0


def cb(data):
    print data.header.stamp.to_sec()
    global prev_time, cur_time, ctr
    prev_time = cur_time
    cur_time = data.wall_time
    if prev_time >= cur_time:
        print 'Time moved backwards, CT: ', cur_time, ' PT: ', prev_time
    ctr += 1
    if ctr % 500 == 0:
        print 'Counter: ', ctr



rospy.init_node('mynode')
sub = rospy.Subscriber('/chai/env/Torus/State', ObjectState, cb, queue_size=100)

while not rospy.is_shutdown():
    a = 0
