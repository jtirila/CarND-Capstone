# CarND Capstone project

**By: Juha-Matti Tirilä**

This is my implementation of the Capstone project of Udacity's CarND.

The task was to implement various nodes on top of ROS to achieve the capability to navigate a vehicle around a
test track.

Some of the most important parts of the software that were written during the project were:

 * Some ROS subscribers and publishers related to waypoint, traffic light and image data processing
 * Logic to control the steering, throttle and brake of the car so it follow the waypoints
 * An image classifier and related infrastructure for detecting red lights

## Some Notes on My Implentation

### The "Team"

Even though the suggestion is to complete the project as a team, I chose to work alone. I had to
complete the work in a narrow timeframe so instead of relying on other people's schedules, I figured it
is best for me to just do it myself.

### The Generic ROS Code and Throttle, Steering Controllers

These pieces of code I wrote independently, loosely following the suggestions made in the project
walkthrough videos. I adjusted the code a bit here and there and used the provided Yaw and PID
controllers. No attempt was made at making the solution any fancier than that.

### The Image Classifier for Detecting Red Lights

I set out for the image classification task with the following plan in mind:

1. Find some suitable labeled traffic light dataset for teaching
1. Also find a nice convolutional image classifier from the TensorFlow model zoo and use it for
   training and inference, instead of defining my own model layer by layer
1. Save the output of the chosen model and use it for inference in the `tl_detection` ROS node.

With this plan in mind, I came across [Alex Lechner's solution](https://github.com/alex-lechner/Traffic-Light-Classification).
Alex had a nicely documented workflow for this exact task. I followed his approach pretty much as is, downloading
the `SSD_inception_v2_coco` model and using it on an AWS instance to train the model. So even though
I followed what Alex had done, I really did replicate all his steps and produced my own frozen inference graph.

### The Image Classifier Inference Workflow

As for the workflow of initializing the TensorFlow model from the frozen graph and the using if for
image classification, I also used the solution Alex Lechner and his team as a reference but heavily
modified the code and implemented parts of the solution independently. Specifically, for locating the closest
traffic light, I made use of the KDTree solution used elsewhere in the project, instead of implementing the
naive loop based optimization solution.

## Conclusion

Even though pressed with time, I really enjoyed the project and working with ROS. I find ROS a really nice
solution for multi-component systems like these, involving message passing between loosely connected parts of
a system. This mechanism of topic subscribers and publishers reminded me heavily of the Signals and Slots
mechanism of the Qt framework.
