Error based ADRC (Active Disturbance Rejection Control) is an control method that estimate extended state of object. That's state consist of total disturbance and state of the object. In the error base concept, state of object is error and its derivatives up to the number of object order.  

Longitudianl controller takes velocity from way point as input and acceleration as output (control signal). 

To create ESO, the order of the object must be know. Result of identification are following:

![alt text](result_of_identification.png){ width=50% height=50% }

The plot shows dynamics of integrating system, so the order of control object is 1 and order of is 2 (order of object + 1).

Controller consist of one main class and second that are responsible of extended state observer. ESO class have method to initialize obserwer, calculate state of ESO and helper functions. Main class despite of the common function for logitudinal controller, have function to calculate control signal. 

There are some plots which compare performance eADRC and PID longitudinal controller.

