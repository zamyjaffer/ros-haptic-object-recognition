#!/usr/bin/env python3

import rospy
from collections import defaultdict
from haptic_object_recognition.msg import experiment_variables
from haptic_object_recognition.msg import plot_accuracy
from haptic_object_recognition.srv import *

#creating dictionaries based on the probabilities I calculated
pA = {
    (1, 3): {'correct': 0.450, 'incorrect': 0.550},
    (1, 5): {'correct': 0.480, 'incorrect': 0.520},
    (1, 7): {'correct': 0.545, 'incorrect': 0.455},
    (1, 9): {'correct': 0.525, 'incorrect': 0.475},
    (1, 15): {'correct': 0.530, 'incorrect': 0.470},
    (2, 3): {'correct': 0.630, 'incorrect': 0.370},
    (2, 5): {'correct': 0.660, 'incorrect': 0.340},
    (2, 7): {'correct': 0.725, 'incorrect': 0.275},
    (2, 9): {'correct': 0.705, 'incorrect': 0.295},
    (2, 15): {'correct': 0.710, 'incorrect': 0.290},
    (3, 3): {'correct': 0.560, 'incorrect': 0.440},
    (3, 5): {'correct': 0.590, 'incorrect': 0.410},
    (3, 7): {'correct': 0.655, 'incorrect': 0.345},
    (3, 9): {'correct': 0.635, 'incorrect': 0.365},
    (3, 15): {'correct': 0.640, 'incorrect': 0.360},
    (4, 3): {'correct': 0.515, 'incorrect': 0.485},
    (4, 5): {'correct': 0.545, 'incorrect': 0.455},
    (4, 7): {'correct': 0.610, 'incorrect': 0.390},
    (4, 9): {'correct': 0.590, 'incorrect': 0.410},
    (4, 15): {'correct': 0.595, 'incorrect': 0.405},
    (5, 3): {'correct': 0.660, 'incorrect': 0.340},
    (5, 5): {'correct': 0.690, 'incorrect': 0.310},
    (5, 7): {'correct': 0.755, 'incorrect': 0.245},
    (5, 9): {'correct': 0.735, 'incorrect': 0.265},
    (5, 15): {'correct': 0.740, 'incorrect': 0.260},
    (6, 3): {'correct': 0.730, 'incorrect': 0.270},
    (6, 5): {'correct': 0.760, 'incorrect': 0.240},
    (6, 7): {'correct': 0.825, 'incorrect': 0.175},
    (6, 9): {'correct': 0.805, 'incorrect': 0.195},
    (6, 15): {'correct': 0.810, 'incorrect': 0.190},
    (7, 3): {'correct': 0.515, 'incorrect': 0.485},
    (7, 5): {'correct': 0.545, 'incorrect': 0.455},
    (7, 7): {'correct': 0.610, 'incorrect': 0.390},
    (7, 9): {'correct': 0.590, 'incorrect': 0.410},
    (7, 15): {'correct': 0.595, 'incorrect': 0.405},
    (8, 3): {'correct': 0.530, 'incorrect': 0.470},
    (8, 5): {'correct': 0.560, 'incorrect': 0.440},
    (8, 7): {'correct': 0.625, 'incorrect': 0.375},
    (8, 9): {'correct': 0.605, 'incorrect': 0.395},
    (8, 15): {'correct': 0.610, 'incorrect': 0.390},
    (9, 3): {'correct': 0.645, 'incorrect': 0.355},
    (9, 5): {'correct': 0.675, 'incorrect': 0.325},
    (9, 7): {'correct': 0.740, 'incorrect': 0.260},
    (9, 9): {'correct': 0.720, 'incorrect': 0.280},
    (9, 15): {'correct': 0.725, 'incorrect': 0.275},
    (10, 3): {'correct': 0.675, 'incorrect': 0.325},
    (10, 5): {'correct': 0.705, 'incorrect': 0.295},
    (10, 7): {'correct': 0.770, 'incorrect': 0.230},
    (10, 9): {'correct': 0.750, 'incorrect': 0.250},
    (10, 15): {'correct': 0.755, 'incorrect': 0.245},
    (11, 3): {'correct': 0.715, 'incorrect': 0.285},
    (11, 5): {'correct': 0.745, 'incorrect': 0.255},
    (11, 7): {'correct': 0.810, 'incorrect': 0.190},
    (11, 9): {'correct': 0.790, 'incorrect': 0.210},
    (11, 15): {'correct': 0.795, 'incorrect': 0.205},
    (12, 3): {'correct': 0.615, 'incorrect': 0.385},
    (12, 5): {'correct': 0.645, 'incorrect': 0.355},
    (12, 7): {'correct': 0.710, 'incorrect': 0.290},
    (12, 9): {'correct': 0.690, 'incorrect': 0.310},
    (12, 15): {'correct': 0.695, 'incorrect': 0.305}
}

#function to predict recognition accuracy using Bayesian network
def predict_bayes_network(stimulus_object, haptic_time):
    p_correct, p_incorrect = 0.0, 0.0
    print(stimulus_object, haptic_time)

    result = pA.get((stimulus_object, haptic_time), None)
    if result:
        p_correct, p_incorrect = result.get("correct", 0.0), result.get("incorrect", 0.0)

    return p_correct, p_incorrect


#function to handle the service request
def handle_request(request):
    #predicting the recognition accuracy using bayesian network
    p_correct, p_incorrect = predict_bayes_network(current_observation['stimulus_object'], current_observation['haptic_time'])

    rospy.loginfo('Recognition accuracy calculated', current_observation)

    return p_correct, p_incorrect


#function to handle the experiment variables received through the topic
def handle_experiment_variables(msg):
    #updating the current observation with the perceived info
    current_observation['stimulus_object'] = msg.stimulus_object
    current_observation['haptic_time'] = msg.haptic_time

accuracy_dict = defaultdict(list)
for stimulus_obj, results in pA.items():
    correct_value = results['correct']
    accuracy_dict[stimulus_obj[0]].append(correct_value)
#print(accuracy_dict)

if __name__ == '__main__':
    #initialising the node
    rospy.init_node('recognition_accuracy_prediction', anonymous=True)

    #initialising the service
    service = rospy.Service('compute_recognition_accuracy', compute_recognition_accuracy, handle_request)
    rospy.loginfo('Ready to compute recognition accuracy')

    #subscribing to the experiment_variables topic
    subscriber = rospy.Subscriber('experiment_variables', experiment_variables, handle_experiment_variables)

    #initialising the current observation
    current_observation = {
        'stimulus_object': 0,
        'haptic_time': 0
    }
    
    pub = rospy.Publisher('plot_accuracy', plot_accuracy, queue_size=10)
    msg = plot_accuracy()
    for i in accuracy_dict.items():
        msg.haptic_time_3 = i[1][0]
        msg.haptic_time_5 = i[1][1]
        msg.haptic_time_7 = i[1][2]
        msg.haptic_time_9 = i[1][3]
        msg.haptic_time_15 = i[1][4]
        rospy.loginfo(msg)
        pub.publish(msg)
   
    rospy.spin()
