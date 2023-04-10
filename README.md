# ROS Haptic Object Recognition

## Description
- This is a ROS project that creates a bayesian network to predict the accuracy of a subject correctly recognising and identifying an object, after a given amount of haptic manipulation time. This project consists of 4 nodes:
  - 'haptic_interaction_generator' which randomly picks an object to be haptically maniplated, as well as the amount of time allowed for haptic manipluation (from a list of 3, 5, 7, 9 & 15 seconds).
  - 'experiment_execution' which subscribes to the topics from the first node and calls node 4 to calculate the probabilities of recognition accuracy. 
  - 'recognition_accuracy_prediction' which implements the 'compute_recognition_accuracy' service and uses a Bayesian network to predict the probability of the recognition accuracy (% liklihood of the identification being correct), given the object and time allowed to haptically manipulate it.
  - 'visualise_results' which subscribes to node 3 to recive the probabilities and plots the results of the experiment in 3 different formats.
- This project also consists of a launch file that starts the nodes and shows graph of this project using rqt_graph.

## Installation
- Download the 'haptic_object_recognition' package into your catkin workspace 
- And then build your workspace by running the following command: 
```
catkin_make
```

## Running
- from your catkin_workspace activate roscore: 
```
roscore
```

- from your catkin_workspace navigate to the scripts folder: 
```
cd /src/haptic_object_recognition/scripts
```
    
- run the following commands from within the srcipts folder for each node:
```
chmod +x haptic_interaction_generator.py
chmod +x experiment_execution.py
chmod +x recognition_accuracy_prediction.py
chmod +x visualise_results.py
```
    
- navigate back to the catkin_ws folder: 
```
cd ~/ catkin_ws
```

- launch the package:
```
roslaunch haptic_object_recognition haptic_object_recognition.launch
```

## Dependencies
- python 3.8.10
- ros-noetic
- random
- matplotlib
