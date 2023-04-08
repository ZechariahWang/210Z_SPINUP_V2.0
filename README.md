
![RecoloredLogo](https://user-images.githubusercontent.com/97078224/230698177-5690229d-7d7e-4f90-89dc-b33ee27bca9b.png)



# Team 210Z 2022-2023 Code

Code Framework for 210Z during the 2022-2023 Vex Robotics Competition.


## Usage

here are some basic project paths:

1: Install the package from this repository. 

2: To set up the components of the robot, navigate to
``` Globals.cpp ```. It will be found within the ``` Miscellaneous ``` folder under ```CoreSystemVitals```. From there, manipulate the ports of your robot accordingly to the corresponding parts within the code.

3: For driver control, go to ```main.cpp``` and locate the ```opControl``` function. This is the main driver function for all opControl related functions within the robot. If you wish to change core driver vital systems, go to ```CoreAssets.cpp``` under ```DriveHandler``` in ```CoreSystemVitals```. For specific in game functions such as flywheels, rollers, etc, go to ```GameAssets.cpp``` under the same corresponding folder.

4: For autonomous control, there are three main components: PID Controller, Odometry Logic, and Algorithm Logic. These are all divided into their own seperate components within the project. Note that certain pieces of logic may require external module and utility functions, which can be found in the ```Modules``` and ```Utility``` folders.

5: For LVGL Embedded System Graphics, all logic will be found in ```main.cpp``` in the ```initialize``` function. Note that declaration for specific components may be found in other paths in the project.


## Authors

- [@ZechariahWang](https://github.com/ZechariahWang)


## Contributors

 - Kevin Zhao
 - Andrew Li



Simulations: https://github.com/ZechariahWang/Motion-Profiling-Algorithm_Simulations
