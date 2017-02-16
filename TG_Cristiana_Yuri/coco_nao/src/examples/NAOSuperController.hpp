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
	DQ_robotics::DQ eff_translation_difference;
	Eigen::Matrix<double,5,1> delta_joints;
	std::ofstream Results_L;
	std::ofstream Results_R;
	ros::Time initialTime;

	NAOSaver Save;
	int fileIndexErrorAbs;
	int fileIndexPosR;
	int fileIndexPosL;
	int fileIndexPosAbs;
	int fileIndexPosRel;
	int fileIndexErrorRel;
    int fileIndexErrorLeft;
    int fileIndexErrorRight;

   	int	fileIndexJacob;
    int fileIndexDelta;
    int fileIndexInitial;
    int fileIndexFinal;


	DQ_robotics::DQ reference_rotation_L_, reference_rotation_R_;

public:	
	cocoNAO();
	~cocoNAO();
	void DQinvKin(const coco_nao::NaoJointPtr& joint_msg);
	void saveData(DQ_robotics::DQ pose, double time);

};
