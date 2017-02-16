Prerequisites
	-Linux System 
	-Matlab
	-DQRobotics: http://dqrobotics.sourceforge.net/
	-naoqi-sdk-2.1.4.13-linux64
	-ROS
	- 
*************************************************************************************************
*												*
*						ROS						*
*												*
*************************************************************************************************
*************************************************************************************************
					Prerequisites
*************************************************************************************************

- Ubuntu 14.04
- Ros Indigo 
- DQRobotics: http://dqrobotics.sourceforge.net/
- naoqi-sdk-2.1.4.13-linux64
- NAOQI Bridge: http://ros-naoqi.github.io/naoqi_driver/

*************************************************************************************************
Configurations
*************************************************************************************************
CMakeLists.txt
Line 5 - Change to your computer's path
*********
Open folder <launch> and alter the file coco_nao.launch:

Line 3 - 
<env name="LD_LIBRARY_PATH" value="$(env LD_LIBRARY_PATH):**PATH TO FOLDER naoqi-sdk-2.1.4.13-linux64/lib**:$(find coco_nao)/libs"/>

**PATH TO FOLDER naoqi-sdk-2.1.4.13-linux64/lib** must be replaced by your folder naoqi-sdk-2.1.4.13-linux64/lib

For Example

<env name="LD_LIBRARY_PATH" value="$(env LD_LIBRARY_PATH):/home/cris/nao/devtools/naoqi-sdk-2.1.4.13-linux64/lib:$(find coco_nao)/libs"/>
***********
Open the file <nodes/nao_subscriber_inv.py>
Line 87 - 
Change the IP to the IP of your robot
**************************************************************************************************	
Running NAOqi
**************************************************************************************************
Open a terminal for the ROS inicialization
	ROS_IP=IPCOMPUTADOR roscore

Open another terminal for inicializing NAODriver
	roslaunch naoqi_driver naoqi_driver.launch nao_ip:=IPROBO roscore_ip:=IOMPUTADOR network_intface:=eth0

Now you are connected to your robot, to turn the arm joints on you must run the command that will call the python script nao_subscriber_inv.py
	roslaunch coco_nao coco_nao.launch
****
To send commands to joints 
rostopic pub /joint_angles naoqi_bridge_msgs/JointAnglesWitpeed [tabs]
