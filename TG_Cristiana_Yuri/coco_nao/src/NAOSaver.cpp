#include "NAOSaver.hpp"

NAOSaver::NAOSaver()
{

}

NAOSaver::~NAOSaver()
{
	int index = (int)(fileDecriptors.size());
	for (int i=0;i<index; i++)
	{
		fileDecriptors[i]->close();
	}
}

int NAOSaver::createFile(std::string fileName)
{
	//std::cout << "Entrou e criou" << fileName << std::endl;
	
	std::ofstream *tmp = new std::ofstream;
	
	tmp->open(fileName.c_str());
	
	int index = (int)(fileDecriptors.size());
	
	fileDecriptors.push_back(tmp);
	return index;
}

void NAOSaver::saveError(Eigen::VectorXd Error, double time, int index)
{
	std::cout << "Save Error" << std::endl;
	*(fileDecriptors[index]) << Error.transpose() << ", " << time << std::endl;
	
}

void  NAOSaver::savePosition(DQ_robotics::DQ Joints, double time, int index)
{

	std::cout << "Save Pos" << std::endl;

	*(fileDecriptors[index]) << Joints << ", " << time << std::endl;
}

void NAOSaver::saveJacob(Eigen::MatrixXd Jacobian, double time, int index)
{
	*(fileDecriptors[index]) << Jacobian << ", " << time << std::endl;
}

void NAOSaver::saveJoints(Eigen::MatrixXd Joints, double time, int index)
{
	*(fileDecriptors[index]) << Joints << ", " << time << std::endl;
}