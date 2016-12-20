#include "nao_joint_publisher.hpp"
#include <vector>
//#include "ArmFunctions.hpp"

struct DataSave
{
	std::vector<double> JointSave;
	double Time; //nano segundos
};

NAOJoints::NAOJoints()
{

	//Results.open("/home/cris/catkin_ws/src/coco_nao/resultados.txt");
	
}

NAOJoints::~NAOJoints()
{
	//Results.close();
}

void NAOJoints::Joints_CallBack( const sensor_msgs::JointStatePtr& js_msg)
{
	//descobrir numero da junta: Usar comando rostopic echo /joint_states
	/*std::cout << js_msg->name[2] << "joint: " << js_msg->position[2] << std::endl; 
	std::cout << js_msg->name[3] << "joint: " << js_msg->position[3] << std::endl;
	std::cout << js_msg->name[4] << "joint: " << js_msg->position[4] << std::endl;
	std::cout << js_msg->name[5] << "joint: " << js_msg->position[5] << std::endl;
	std::cout << js_msg->name[6] << "joint: " << js_msg->position[6] << std::endl;*/
	
	//std::cout << "Saving data... " << std::endl;
	

	std::vector<double> Joints;
	
	std::string Chain;	
	Chain = "Arm";

	DataSave jointResults;

	/*jointResults.JointSave.push_back(js_msg->position[2]);
	jointResults.JointSave.push_back(js_msg->position[3]);
	jointResults.JointSave.push_back(js_msg->position[4]);
	jointResults.JointSave.push_back(js_msg->position[5]);
	jointResults.JointSave.push_back(js_msg->position[6]);
	jointResults.JointSave.push_back(js_msg->position[20]);
	jointResults.JointSave.push_back(js_msg->position[21]);
	jointResults.JointSave.push_back(js_msg->position[22]);
	jointResults.JointSave.push_back(js_msg->position[23]);
	jointResults.JointSave.push_back(js_msg->position[24]);*/


	ros::Time timeNAO = ros::Time::now();

	timeNAO.sec -= initialTime.sec;

	jointResults.Time = (timeNAO.nsec/1000000 + timeNAO.sec*1000);

	//std::cout << timeNAO.sec*1000 << std::endl;
	//std::cout << timeNAO.nsec/1000000 << std::endl;

	//Results << jointResults.JointSave[0] << ", " << jointResults.JointSave[1] << ", " << jointResults.JointSave[2] << ", " 
	//<< jointResults.JointSave[3] << ", " << jointResults.JointSave[4] << ", " << jointResults.JointSave[5] << ", " << jointResults.JointSave[6] << 
	//", " << jointResults.JointSave[7] << ", " << jointResults.JointSave[8] << ", " << jointResults.JointSave[9] << ", " << std::fixed << jointResults.Time << std::endl;


	if(Chain == "LArm")
	{
		Joints.push_back(js_msg->position[2]);
		Joints.push_back(js_msg->position[3]);
		Joints.push_back(js_msg->position[4]);
		Joints.push_back(js_msg->position[5]);
		Joints.push_back(js_msg->position[6]);
	}
	else if(Chain == "RArm")
	{
		Joints.push_back(js_msg->position[20]);
		Joints.push_back(js_msg->position[21]);
		Joints.push_back(js_msg->position[22]);
		Joints.push_back(js_msg->position[23]);
		Joints.push_back(js_msg->position[24]);
	}
	else if (Chain == "Arm")
	{
		Joints.push_back(js_msg->position[2]);
		Joints.push_back(js_msg->position[3]);
		Joints.push_back(js_msg->position[4]);
		Joints.push_back(js_msg->position[5]);
		Joints.push_back(js_msg->position[6]);
		Joints.push_back(js_msg->position[20]);
		Joints.push_back(js_msg->position[21]);
		Joints.push_back(js_msg->position[22]);
		Joints.push_back(js_msg->position[23]);
		Joints.push_back(js_msg->position[24]);

	}
	coco_nao::NaoJoint msg;
	msg.joints = Joints;
	msg.chain = Chain;

	pub.publish(msg);
	//ROS_INFO("%s", js_msg->name(0));
}

void NAOJoints::mainRoutine(int argc, char **argv)
{
	ros::init(argc, argv, "Client_GetJoints_NAO");

	ros::NodeHandle node;

	initialTime = ros::Time::now();



    ROS_INFO("Nao entrou");

   //( joint_angles_topic_, 10, &TeleopSubscriber::joint_angles_callback, this );
    ros::Subscriber sub = node.subscribe("/joint_states", 1, &NAOJoints::Joints_CallBack, this);

    pub = node.advertise<coco_nao::NaoJoint>("NAO_JOINTS", 1);


    ros::spin();
 
}



int main(int argc, char **argv)
{

	NAOJoints NAO;

    

    NAO.mainRoutine(argc, argv);

	

	return 0;
}
    

