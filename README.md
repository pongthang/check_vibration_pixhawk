# check_vibration_pixhawk
Using this program, vibration of individual motors can be checked.

# Why do we need this program?

A drone or UAV has many sensors to estimate its own current position, attitude, velocity, etc. According to that it decides its action.
<p>
Unfortunately, vibrations in the drone or UAV mess up the data coming from the sensors. Due to this, the estimators give wrong estimation
of the current conditions of the drone or UAV which results in crash or very unstable flight if the viration is too high.

</p>

<p>
So, to solve this problem, this program is written in order to check the vibration of individual motor. Here the acceleration along the different 
axes are calculated using accelerometer. The acceleration along the z-axis must be 9.8 to 10 m/s^2 in very less vibration.
</p>

# How to use?

<p>

* Add this folder in the pixhawk source code inside the example folder ../PX4-Autopilot/src/examples.

Source code is available at https://github.com/PX4/PX4-Autopilot.git

* Build the source code and upload to pixhawk. All the steps for building and uploading is available in http://docs.px4.io/main/en/dev_setup/building_px4.html

* Open console and run the program. Example : check_vibration 3000 <br>
3000 is the number of messages you want to see in the console.

* Move the sticks of the RC transmitter to change the motor number. According to the motor number, different motors can be tested.
* Lower the throttle. Then move the roll stick to extreme right. Now increase the throttle.  You will see the motor corresponding to the stick will start rotation.
* Similary lower the throttle and change to other motor by moving the sticks other than throttle. And repeat the process.
<p>
Check the value of the vibration in z-axis . For good vibration , the value must vary from 9 to 10 only or 9 to 10.5.
Reduce your drone vibration by balancing the propellers or tighten loose parts. 
</p>
