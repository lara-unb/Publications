#pragma once

#include "DQ.h"
#include "DQ_kinematics.h"

#include <vector>
#include <string>
#include "ros/ros.h"
#include <Eigen/Dense>
#include <fstream>

class NAOSaver
{
private:
	std::vector<std::ofstream*> fileDecriptors;

public:
	NAOSaver();
	~NAOSaver();
	int createFile(std::string fileName);
	void saveError(Eigen::VectorXd, double time, int index);
	void savePosition(DQ_robotics::DQ Joints, double time, int index);
	void saveJacob(Eigen::MatrixXd Jacobian, double time, int index);
	void saveJoints(Eigen::MatrixXd Joints, double time, int index);



};