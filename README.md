# MotorVibe
Scripts for automatically driving UAV brushless motor ESCs using PWM signals from a Linux sysfs pwm interface.  Also contains a script for calculating and comparing basic stats on accelerometer data pulled from PX4 ulog files
and/or mavlink ```VIBRATION``` messages.

# Setup
- PX4 or Ardupilot Firmware-loaded Autopilot mounted to the armature near a brushless motor to be assayed for its
  intrinsic vibration levels.  In my case, I used the [mRo PixRacer R15](https://store.mrobotics.io/mRo-PixRacer-R15-Official-p/auav-pxrcr-r15-mr.htm) mounted to the armature of a multicopter UAV running the [PX4 v1.9.2](https://github.com/PX4/Firmware/tree/v1.9.2) 
  firmware.
- Microcontroller with Linux sysfs pwm interface to enable chipset PWM channels and generate PWM signals of a given
  period, duty cycle, and polarity. In my case I used a [NanoPi
  NEO2](https://www.friendlyarm.com/index.php?route=product/product&product_id=180) microcontroller, as it is an camera
  interface board I plan on using on a UAV.
- Ground controller software (GCS) running on a microcontroller that can interface via USB or a supported wireless interface
  to communicate using the Mavlink protocol with an autopilot.  In my case, I used my fork of
  [MAVProxy](https://github.com/bliptrip/MAVProxy) running on a Raspberry Pi microcontroller board, which is directly
  connected to the autopilot using a USB cable.  The GCS can run on the same microcontroller system as the one driving
  the motors through the PWM interface, or separately.


# Tree Layout
- ```config``` - This folder contains motor specific pwm values to loop through when testing DC brushless motors.  Note that
  pairs of files should have an equal number of entries (lines) and together should have approximately the same RPMs at a given
  applied input voltage.  The values on each line is a PWM duty cycle value, in nanoseconds.  To generate these values,
  I used the KV value (_k_onstant velocity, or rotations per minute, per input volt applied) between the two motors to generate a set of equivalent RPM
  rates with different PWM values.
    - ```sunnysky.txt``` - Sunnysky Model V5208-10 KV: 340
    - ```tmotor.txt``` - TMotor MN505-S KV: 380
- ```params``` - This folder contains the parameter files for given PX4 or Ardupilot-based firmware on a given hardware platform autopilot.  They are mostly used to change the settings on an autopilot to use it's internal IMUs to measure motor vibrations.
- ```Makefile``` - This makefile simply gives examples of how I invoked the included python scripts to both drive motors
  and generate comparative plots/data for assessing the vibrational dynamics of two different motors.
-  ```motor.py``` - The python script that cycles through different PWM values based on one of the files in the
   ```config/``` folder, driving the motors at different speeds.  It also includes logic for sending mavlink commands to
   the autopilot to Arm/Disarm (needed to generate PX4 logs), along with sending RC overrides to drop/set the throttle
   to allow Arming, and set RC channel 5 to put the autopilot in 'Manual' mode.
- ```plot_vibrations.py``` - This script uses the python bokeh visualization package to generate comparative plots of
  the vibration power spectral density graphs.  This code was shamelessly cobbled together from PX4's [flight
  review](https://github.com/PX4/flight_review), and requires one to have downloaded PX4 ulog files for each arm/disarm
  cycle of the motors.
- ```summarize_vibration_stats.py``` - This script generates an output csv file to show comparative accelerometer
  readings on the x, y, and z axes of autopilot's IMU.  This code uses data compiled from ```VIBRATION``` mavlink messages.

# Gotchas
- It is a good idea to erase the logfiles between runs on autopilot's microSD card.
- For some reason software rebooting (using mavlink command or through GCS) an autopilot with PX4 firmware seems to allow it to correctly respond to the ```motor.py``` script's arm/disarm commands.
- When using a [MAVProxy](ihttps://github.com/bliptrip/MAVProxy) GCS that directly interfaces with the autopilot, it is useful to disable some of the default modules (rc, etc.) to prevent it from sending it's own RC overrides that are
different from ```motor.py```.  For example, on my setup, I invoked **Mavproxy** with 
```/usr/local/bin/mavproxy.py --master=/dev/ttyACM0,24000000 --out=udpin:0.0.0.0:14551 --aircraft=bliptrip -a --mav20 --default-modules="param,arm,output,cmdlong"```
- It is generally more helpful to manually download the PX4 logs of the microSD card after running through a set of motor speeds, over downloading them in realtime over mavlink.  This has to do with the fact
that the code for downloading the logs over mavlink isn't robust, and also with the fact that these links are often too slow to make it useful.

# To Do's
- Modify the ```motor.py``` script to cycle through a set of desired 'RPM' values, and change the config files to
  something like a YAML format that describe features of each motor, specifically their KV values (that allow conversion
  of an input voltage to an output RPM per voltage applied).  Note: If I decide to change the input config from PWM
  values to RPM values, then I will need to also specify the input voltage to the Motor ESCs as a command-line argument.
- Better debug/understand the reasons why the PX4 autopilot won't sometimes respond to arm requests, either from
  failure to receive/register throttle down on RC override, or otherwise.
- Build in a way to check the arming status of the autopilot before running a motor spinup test, so that we know a
  separate, corresponding logfile is being generated for this motor spin rate.  Retry so many times and error out if it
  fails to arm.
- Write a script to automatically pull log files off the microSD card and put in the correct folders for later assement.
  This will require getting the filename of the logfiles from each arm/disarm cycle.
- Modify the plot_vibrations.py to show x, y, and z acceleration values, in addition to the power spectral density of
the combined axes.
- Modify the plot_vibrations.py to allow it to dynamically render graphs as the PX4 ulog data comes down the mavlink
  pipeline (will require a fast mavlink connection).
   
