#!/usr/bin/env python3

import rospy
import random 
from haptic_object_recognition.msg import experiment_variables
from haptic_object_recognition.msg import recognition_accuracy
from haptic_object_recognition.srv import *

class experimentExecution:

    def __init__(self):
        #initialising the node
        rospy.init_node('experiment_execution')
        #waiting for the perdict robot expression service to start
        rospy.wait_for_service('compute_recognition_accuracy')
        #creating a proxy for the service
        self.compute_recognition_accuracy = rospy.ServiceProxy('compute_recognition_accuracy', compute_recognition_accuracy)
        #subscribing to the experiment_variables topic
        self.experiment_variable_sub = rospy.Subscriber('experiment_variables', experiment_variables, self.callback)
        #publishing the recognition_accuracy topic
        self.recognition_accuracy_pub = rospy.Publisher('recognition_accuracy', recognition_accuracy, queue_size=0)
        
    def callback(self, msg):
        #initialising probabilities
        p_correct, p_incorrect = 0.0, 0.0
        id = msg.id
        
        #getting probabilities from the service
        if msg.stimulus_object != 0 and msg.haptic_time != 0:
            try:
                #getting the response
                response = self.compute_recognition_accuracy(msg.stimulus_object, msg.haptic_time)
                #assigining variables
                p_correct, p_incorrect = response.p_correct, response.p_incorrect
                #logging probabilities
                rospy.loginfo("Correct: %f, Incorrect: %f", p_correct, p_incorrect)
                
                #publishing robot info message
                recognition_accuracy_msg = recognition_accuracy()
                recognition_accuracy_msg.id = id
                recognition_accuracy_msg.p_correct = p_correct
                recognition_accuracy_msg.p_incorrect = p_incorrect
                self.recognition_accuracy_pub.publish(recognition_accuracy_msg)
                
            except rospy.ServiceException as e:
                print("Service call failed: ", e)

if __name__ == '__main__':
    try:
        experiment_execution = experimentExecution()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
