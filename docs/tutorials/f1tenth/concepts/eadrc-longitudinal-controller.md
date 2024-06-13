# Error-based ADRC logitudinal controller

Error-based ADRC (Active Disturbance Rejection Control) is a control method that estimates the extended state of an object. This state consists of the total disturbance and the state of the object. In the error-based concept, the state of the object is represented by the error and its derivatives up to the order of the object. 

Longitudianl controller takes velocity from waypoint as input and produces acceleration as output (control signal).

To create an ESO, the order of the object must be known. The results of the identification are as follows:

<img src="https://github.com/Karol-Debski/png_to_autoware_2/blob/devel/result_of_identification.png" alt="Description" style="width:50%;">

The plot shows the dynamics of an integrating system, so the order of the control object is 1 and the order of the ESO is 2 (order of the object + 1).

The controller consists of one main class and a second class that is responsible for the extended state observer (ESO). The ESO class has methods to initialize the observer, calculate the state of the ESO and provide helper functions. The main class, in addition to common functions for the longitudinal controller, has a function to calculate the control signal.

There are some plots that compare the performance of the eADRC and PID longitudinal controllers.

Error based controller ADRC:

<img src="https://github.com/marcelq11/pngtoautoware/blob/main/02_6_1.png?raw=true" style="width:50%;">

PID controller:

<img src="https://github.com/marcelq11/pngtoautoware/blob/main/pid.png?raw=true" style="width:50%;">

The video that present working controller eADRC:

https://www.youtube.com/watch?v=GPXSC7kA7KQ
