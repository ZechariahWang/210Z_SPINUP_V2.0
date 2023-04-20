![RecoloredLogo](https://user-images.githubusercontent.com/97078224/230698177-5690229d-7d7e-4f90-89dc-b33ee27bca9b.png)


# Team 210Z 2022-2023 Framework

Code Framework for 210Z during the 2022-2023 Vex Robotics Competition.

- 6x World Championship Qualifiers
- 15x Honours Received
- 2022-2023 Alberta Provincial Champions
- 2022-2023 Mecha Mayhem Signature Event Champions
- Peak 6th in Canada, 57th in the world.




## Project Structure
The project is divided into three major sections: Core System Assets, Scripts, and Modules. The Core System Assets are the driving functions of the robot that control the fundamental components of the game logic, such as driver control, autonomous, and so on. The Scripts are the courses that the robot can take throughout the autonomous phases of the competition. These may include abilities, in-game autons, etc. The module section contains helper functions that can be called on the logic within the Core System Assets.

## Prerequisites
In order for the project to work, you must have:
- PROS (Purdue University)
- A physical robot with VEX V5 hardware/firmware


## Project Usage
1: Install the package from this repository using ```git clone https://github.com/ZechariahWang/210Z_SPINUP_V2.0.git```

2: To set up the components of the robot, navigate to
``` Globals.cpp ```. It will be found within the ``` Miscellaneous ``` folder under ```CoreSystemVitals```. From there, manipulate the ports of your robot accordingly to the corresponding parts within the code.

3: For driver control, go to ```main.cpp``` and locate the ```opControl``` function. This is the main driver function for all opControl related functions within the robot. If you wish to change core driver vital systems, go to ```CoreAssets.cpp``` under ```DriveHandler``` in ```CoreSystemVitals```. For specific in game functions such as flywheels, rollers, etc, go to ```GameAssets.cpp``` under the same corresponding folder.

4: For autonomous control, there are three main components: PID Controller, Odometry Logic, and Algorithm Logic. These are all divided into their own seperate components within the project. Note that certain pieces of logic may require external module and utility functions, which can be found in the ```Modules``` and ```Utility``` folders.

5: For LVGL Embedded System Graphics, all logic will be found in ```main.cpp``` in the ```initialize``` function. Note that declaration for specific components may be found in other paths of the project. The current setup of the interface is in 4 components: Game, Sensor, Auton, Misc. Each section displays data about it's corresponding section, but may connect to other parts of the project. The biggest notable example of this is the autonomous selector. To edit or change logic within the selector, navigate to ```InitializeModule.cpp``` in the ```OnStartupModule``` folder.


## Authors

- [@ZechariahWang](https://github.com/ZechariahWang)


## Contributors

 - Kevin Zhao
 - Andrew Li


Simulations: https://github.com/ZechariahWang/Motion-Profiling-Algorithm_Simulations
