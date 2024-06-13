


# Msc data science Project : Real-time implementation of foot-mounted inertial navigation

## project description
In this project you will get to perform a real-time implementation of a foot-mounted inertial navigation algorithm. The algorithm could be implemented by collecting sensor data using either USB wires or Bluetooth. Different visualisation features could be integrated. You could use Python or, for example, a C family language. Sensors will be provided. The algorithm itself is already available, although perhaps not in a language suitable for the real-time implementation.


## resources
Here is the website where you can find resources for the IMU that I gave you: https://inertialelements.com/ You have a MIMU22BL.

Here are two articles that describe the ZUPT-aided INS algorithm: 
https://ieeexplore.ieee.org/abstract/document/6236875
https://ieeexplore.ieee.org/abstract/document/7275464/

You will be able to find some code here, for example, at the bottom: http://www.openshoe.org/?page_id=54

This seems to be a Python version: https://github.com/utiasSTARS/pyshoe
You only need to care about the ZUPT-aided INS algorithm, not the robust version of the Zero-Velocity Detection. 

This is the best paper on zero-velocity detection: https://ieeexplore.ieee.org/abstract/document/5523938.

There are several possible data science extensions we could discuss if you like, but it might be best for you to read up on the basics first. Here is an earlier proposal of a student project that I wrote that never materialized:

The idea of foot-mounted inertial navigation is to compute position estimates by integrating measurements from inertial sensors and applying so-called zero-velocity updates to constrain the position drift whenever the foot is stationary. A zero-velocity detector is used to identify the sampling instances when the foot is stationary. One problem with the standard zero-velocity detector is that the optimal detection threshold is dependent on, for example, the user's gait speed. Therefore, several methods have been proposed for designing zero-velocity detectors that can adapt to gait conditions. The aim of this project is to extend existing adaptive, backward-looking, Bayesian zero-velocity detectors, in order to create a forward-looking detector that makes its decisions by jointly considering the detection statistics at a window of sampling instances. The project would benefit from prior experience in optimization. 
"# real-time-implementation-of-inertia-foot-mounted-navigation-system" 
