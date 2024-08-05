# Overview
The project consist of two parts: 

- 3D Slicer ROS Module, and  
- AMBF simulation plugin of a robot for 3D Slicer. 

The 3D Slicer ROS Module handles the information and communication via ROS. The AMBF simulator Plugin is capable of publishing robot states and robot visualization materials to ROS for the 3D Slicer Module to accquire.

![image](https://user-images.githubusercontent.com/60408626/207697152-3285fb40-c3dd-4dbf-838d-3d9a42558848.png)


## 1.Installation

### 1.1 Install ROS Noetic
[See ROS Wiki Link](http://wiki.ros.org/noetic/Installation/Ubuntu)

### 1.2 Install and build AMBF2.0
Clone and build AMBF2.0  
```bash
git clone https://github.com/WPI-AIM/ambf.git
cd ambf
git checkout -b ambf-2.0 origin/ambf-2.0
git pull
```
Build and source ambf (make sure you're on branch ambf-2.0 before building) as per the instructions on [AMBFs wiki](https://github.com/WPI-AIM/ambf/wiki/Installing-AMBF.)

### 1.3 Install and build 3D Slicer
See build instructions [Here](https://slicer.readthedocs.io/en/latest/developer_guide/build_instructions/index.html)

### 1.4 Clone and build Slicer_ROS_Extension with AMBF
#### 1.4.1 Clone the repo
```bash
git clone https://github.com/LCSR-CIIS/Slicer_ROS_Extension.git
```
#### 1.4.2 Build Slicer_ROS_Extension
```bash
cd 3D-Slicer_ROS_Module_with_AMBF
mkdir Slicer_ROS_Extension-debug
cmake -DCMAKE_BUILD_TYPE:STRING=Debug -DSlicer_DIR:PATH=/path/to/Slicer-SuperBuild-Debug/Slicer-build ../Slicer_ROS_Extension
make
cd ../
```
#### 1.4.3 Build AMBF_Plugin_3DSlicer
```bash
cd AMBF_Plugin_3DSlicer
mkdir build
cd build 
cmake ..
make
```

If no error is shown, the the installation is done.

## 2.Running the 3D Slicer ROS Extension with AMBF
In order for the 3D Slicer ROS Extension and AMBF to work, `roscore` needs to be running. 
```bash
roscore
```
### 2.1 Launch the AMBF Simulator with the Slicer Plugin
```bash
cd <path-to-ambf>/bin/lin-x86_64
./ambf_simulator --launch_file <path-to-3d-slicer-ros-module-with-ambf>/AMBF_Plugin_3DSlicer/ADF/launch.yaml -l 0,2
```
Note that the parameter in the last line `-l 0,2` means to launch the simulation world with item 0 and item 2. Items are defined in the `launch.yaml` file. In this case, 0 is the surgical robot system and 2 is a skull model.
If successful, you should be able to see the AMBF simulation window pop up with a Surgical Robot and a Skull model.
![image](https://user-images.githubusercontent.com/60408626/207698307-b47efe17-1fce-42ff-b9ab-5ddc3f755514.png)

### 2.2 Launch the 3D Slicer
Navigate back to the project directory
```bash
cd Slicer_ROS_Extension-debug
./SlicerWithSlicerROSExtension
```
![image](https://user-images.githubusercontent.com/60408626/207698613-8c10cc76-f004-4f39-8d4a-f31c75f0bc7e.png)

After the 3D Slicer is launch, click on the drop down manual, select Examples -> ROS1_Module.
![image](https://user-images.githubusercontent.com/60408626/207699094-70fca7cb-68b4-43eb-a8c1-0d7b4d899982.png)
In the AMBF simulator, press P to enable Pick tool. You can drag items around by using mouse.
You should now see the scene is now in sync with the AMBF simulator.
![2022-12-14 14-46-48屏幕截图](https://user-images.githubusercontent.com/60408626/207699780-75d0926c-fbce-4421-a7e3-c1a33be1dca1.png)


