#pragma once

//includes DQ
#include "../DQ.h"
#include "../DQ_kinematics.h"

#include "ros/ros.h"
#include "coco_nao/NaoJoint.h" 
#include "coco_nao/MoveJoint.h"
#include <Eigen/Dense>
#include <fstream>
#include "../NAOSaver.hpp"


class cocoNAO
{
private:
	ros::NodeHandle nodeHandle;
	ros::Publisher pub;
	int contadorVergonha;
	DQ_robotics::DQ obj_pose;
	//DQ_robotics::DQ eff_
	float  eff_translation_difference;
	Eigen::Matrix<double,5,1> delta_joints;
	std::ofstream Results;
	ros::Time initialTime;

	NAOSaver Save;
	int fileIndexError;
	int fileIndexPosR;
	int fileIndexPosL;
	float distance;
	int	fileIndexJacob;
    int fileIndexDelta;
    int fileIndexInitial;
    int fileIndexFinal;

public:	
	cocoNAO();
	~cocoNAO();
	void DQinvKin(const coco_nao::NaoJointPtr& joint_msg);
	void saveData(DQ_robotics::DQ pose, double time);

};
