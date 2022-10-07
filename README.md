# Ks114_sonar

## Structure of the module

### ks114_sonar

- Base ks114 sonar class
- Bridge the communication between the sonar and the host via serial RS485
- Handle sonar sensor's state such as started or errored
- Get connected sonar's configurations
- Distance measurement

### ks114_sonar_utility

- This utility tool is built on top of ks114_sonar library
- The configurable parameters:
   1. Address
   2. Communication baud rate
   3. Noise suppression level
   4. Beam angle
   5. Quick detection

### sonars_manager

- This package handles multiple sonars from different vendors
- It depends on ROS infrastructures
- Simple signal filters are implemented
- Sonars data is broadcasted to ROS topics with multiple formats
- Dynamic reconfiguration is enabled for filter usage and detection mode

