#!/usr/bin/env python3

import rospy
import random
from haptic_object_recognition.msg import experiment_variables

def haptic_interaction_generator():
    #creating a publisher for the topic
    pub = rospy.Publisher('experiment_variables', experiment_variables, queue_size=0)
    #initialising the node
    rospy.init_node('haptic_interaction_generator', anonymous=True)
    rate = rospy.Rate(1/10) #once every 10 seconds
    #initialising the id
    interaction_id = 1
    
    while not rospy.is_shutdown():
        #generating random ints for variables
        stimulus_object = random.randint(1, 12)
        haptic_time_options = [3, 5, 7, 9, 15]
        haptic_time = random.choice(haptic_time_options)
        
        #creating variable message
        msg = experiment_variables()
        msg.id = interaction_id
        msg.stimulus_object = stimulus_object
        msg.haptic_time = haptic_time
        rospy.loginfo(msg)
        pub.publish(msg)
        
        #incrementing the id
        interaction_id += 1
        
        #delay
        rate.sleep()

if __name__ == '__main__':
    try:
        haptic_interaction_generator()
    except rospy.ROSInterruptException:
        pass
