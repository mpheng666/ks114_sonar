# Ks114_sonar

## Structure of the module

### ks114_sonar

- Base ks114 sonar class
- Bridge the communication between the sonar and the host via serial RS485
- Handle sonar sensor's state such as started or errored
- Get connected sonar's configurations
- Distance measurement

### ks114_sonar_utility

1. This utility tool is built on top of ks114_sonar library
2. The configurable parameters:
   1. Address
   2. Communication baud rate
   3. Noise suppression level
   4. Beam angle
   5. Quick detection

### sonars_manager

1. This package handles multiple sonars from different vendors
2. It depends on ROS infrastructures
3. Simple signal filters are implemented
4. Sonars data is broadcasted to ROS topics with multiple formats
5. Dynamic reconfiguration is enabled for filter usage and detection mode
