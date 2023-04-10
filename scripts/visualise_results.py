#!/usr/bin/env python3

import rospy
from matplotlib import pyplot as plt
import matplotlib as mpl
from collections import defaultdict
from haptic_object_recognition.msg import plot_accuracy

def visualise_results():

    #init node
    rospy.init_node('visualise_results', anonymous=True)
    
    #dict of cpt
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
    
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(17, 5))
    
    #fetching data from dict for accuracy by object and averaging
    avg_correct_by_object = {}
    for obj in range(1, 13):
        correct_by_haptic_time = []
        for time in [3, 5, 7, 9, 15]:
            if (obj, time) in pA:
                correct_by_haptic_time.append(pA[(obj, time)]['correct'])
                avg_correct_by_object[obj] = sum(correct_by_haptic_time) / len(correct_by_haptic_time)
    
    #avg accuracy by Object Stimulus
    x1 = sorted(avg_correct_by_object.keys())
    y1 = [avg_correct_by_object[key] for key in x1]
    colour_dict = {1: 'tab:blue', 2: 'tab:orange', 3: 'tab:green', 4: 'tab:red', 5: 'tab:purple', 6: 'tab:brown', 7: 'tab:pink', 8: 'tab:grey', 9: 'tab:olive', 10: 'tab:cyan', 11: 'royalblue', 12: 'orange'}
    colours = [colour_dict[obj] for obj in x1]
    ax1.bar(x1, y1, color=colours)
    ax1.axis(ymin=0,ymax=1)
    ax1.set_xlabel('Stimulus Object')
    ax1.set_ylabel('Average Recognition Accuracy (% correct)')
    ax1.set_title('Average Recognition Accuracy by Stimulus Object', fontsize = 10)
    
    #fetching data from dict for accuracy by object
    accuracy_by_object = {}
    for stimulus_obj, results in pA.items():
        correct_value = results['correct']
        if stimulus_obj[0] not in accuracy_by_object:
            accuracy_by_object[stimulus_obj[0]] = [correct_value]
        else:
            accuracy_by_object[stimulus_obj[0]].append(correct_value)
    
    #fecting accuracy by haptic exploration time and averaging
    avg_correct_by_time = {}
    
    for time in [3, 5, 7, 9, 15]:
        correct_by_object = []
        for obj in range(1, 13):
            if (obj, time) in pA:
                correct_by_object.append(pA[(obj, time)]['correct'])
                avg_correct_by_time[time] = sum(correct_by_object) / len(correct_by_object)    
    #accuracy by object by haptic time
    for stimulus_object, values in accuracy_by_object.items():
        colour = colour_dict[stimulus_object]
        ax2.plot(values, label=f"Stimulus Object {stimulus_object}", color=colour, marker='o')
    ax2.axis(ymin=0,ymax=1)
    ax2.set_xlabel('Haptic Exploration Time (secs)')
    ax2.set_ylabel('Average Recognition Accuracy (% correct)')
    ax2.legend(loc = 'lower right', fontsize = 7)
    ax2.set_title('Recognition Accuracy by Haptic Time & Stimulus Object', fontsize = 10)
    xlabels = [3, 5, 7, 9, 15]
    ax2.set_xticks(range(5))
    ax2.set_xticklabels(xlabels)

    #plotting avg Accuracy by Haptic Exploraction Time
    x3 = list(avg_correct_by_time.keys())
    y3 = list(avg_correct_by_time.values())
    ax3.plot(x3, y3, marker='o')
    ax3.axis(ymin=0,ymax=1)
    ax3.set_xlabel('Haptic Exploration Time (secs)')
    ax3.set_ylabel('Average Recognition Accuracy (% correct)')
    ax3.set_title('Average Recognition Accuracy by Haptic Time', fontsize = 10)

    #show the plot
    plt.ion()
    plt.pause(0.1)
    plt.show()
    fig.savefig('results.png') 

if __name__ == '__main__':
    subscriber = rospy.Subscriber('plot_accuracy', plot_accuracy, visualise_results)
    visualise_results()
    rospy.spin()
