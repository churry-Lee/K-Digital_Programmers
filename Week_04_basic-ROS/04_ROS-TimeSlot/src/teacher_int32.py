#!/usr/bin/env python

import rospy
import timeit
from std_msgs.msg import Int32

rospy.init_node('teacher')
pub = rospy.Publisher('msg_to_students', Int32, queue_size = 0)

def do_job(iter):
    for i in range(0, iter):
        i += 1
        pub.publish(i)

r = input('input rate > ')
rate = rospy.Rate(r)

num = input('input counter num> ')
print '\n'

total = 0

for _ in range(r):
    start_send = timeit.default_timer()
    do_job(num)
    end_send = timeit.default_timer()
    send = end_send - start_send
    print 'Send time> %.4f sec'%(send)

    rate.sleep()
    end_sleep = timeit.default_timer()
    sleep = end_sleep - end_send
    print 'Sleep time> %.4f sec'%(sleep)

    print 'Send + Sleep time> %.4f sec'%(send + sleep), '\n'

    total += (send + sleep)

print '-----------------------'
print 'Total time> %.4f sec'%(total)
print '-----------------------'
