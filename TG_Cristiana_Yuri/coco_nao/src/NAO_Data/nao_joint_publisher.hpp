#include "ros/ros.h"
#include "coco_nao/NaoJoint.h"
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <string>
#include <fstream>

class NAOJoints
{
public:
	NAOJoints();
	~NAOJoints();
	void Joints_CallBack( const sensor_msgs::JointStatePtr& js_msg);
	void mainRoutine(int argc, char **argv);
	void publishService();

private:
	
	ros::Publisher pub;
	std::ofstream Results;
	ros::Time initialTime;


};