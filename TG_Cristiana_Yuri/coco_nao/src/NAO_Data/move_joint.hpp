#include "ros/ros.h"
#include "coco_nao/MoveJoint.h"
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <iostream>
#include <string>

class moveJoint
{
public:
	moveJoint();
	void MoveJoints_CallBack( const coco_nao::MoveJointPtr& js_msg);
	void mainRoutine(int argc, char **argv);
	void publishService();
	void initialJoints();

private:
	
	ros::Publisher pub;
	ros::NodeHandle node;

};