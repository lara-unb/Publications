#include "move_joint.hpp"


moveJoint::moveJoint()
{
	pub = node.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("/joint_angles", 1);

}

void moveJoint::MoveJoints_CallBack( const coco_nao::MoveJointPtr& joint_msg)
{

	//Getting Arm Data
	std::vector<std::string> JointNames;
	std::vector<float> JointAngles;

	std::string Chain = joint_msg->chain;

	if(Chain == "RArm")
	{
		JointNames.push_back("RWristYaw");
		JointAngles.push_back(joint_msg->new_joints[4]);

		JointNames.push_back("RElbowRoll");
		JointAngles.push_back(joint_msg->new_joints[3]);

		JointNames.push_back("RElbowYaw");
		JointAngles.push_back(joint_msg->new_joints[2]);

		JointNames.push_back("RShoulderRoll");
		JointAngles.push_back(joint_msg->new_joints[1]);

		JointNames.push_back("RShoulderPitch");
		JointAngles.push_back(joint_msg->new_joints[0]);
	}

	else if(Chain == "LArm")
	{
		JointNames.push_back("LWristYaw");
		JointAngles.push_back(joint_msg->new_joints[4]);

		JointNames.push_back("LElbowRoll");
		JointAngles.push_back(joint_msg->new_joints[3]);

		JointNames.push_back("LElbowYaw");
		JointAngles.push_back(joint_msg->new_joints[2]);

		JointNames.push_back("LShoulderRoll");
		JointAngles.push_back(joint_msg->new_joints[1]);

		JointNames.push_back("LShoulderPitch");
		JointAngles.push_back(joint_msg->new_joints[0]);
	}
	else if(Chain == "Arm")
	{
		JointNames.push_back("LWristYaw");
		JointAngles.push_back(joint_msg->new_joints[4]);

		JointNames.push_back("LElbowRoll");
		JointAngles.push_back(joint_msg->new_joints[3]);

		JointNames.push_back("LElbowYaw");
		JointAngles.push_back(joint_msg->new_joints[2]);

		JointNames.push_back("LShoulderRoll");
		JointAngles.push_back(joint_msg->new_joints[1]);

		JointNames.push_back("LShoulderPitch");
		JointAngles.push_back(joint_msg->new_joints[0]);

		JointNames.push_back("RWristYaw");
		JointAngles.push_back(joint_msg->new_joints[9]);

		JointNames.push_back("RElbowRoll");
		JointAngles.push_back(joint_msg->new_joints[8]);

		JointNames.push_back("RElbowYaw");
		JointAngles.push_back(joint_msg->new_joints[7]);

		JointNames.push_back("RShoulderRoll");
		JointAngles.push_back(joint_msg->new_joints[6]);

		JointNames.push_back("RShoulderPitch");
		JointAngles.push_back(joint_msg->new_joints[5]);
	}

	
	//JointAngles.push_back(0.5);

	
	

	//std::cout << Chain << std::endl; 
	/*Matrix<double,5,1> initial_joints_nao;
	
	initial_joints_nao(0, 0) = joint_msg->joints[0]; 
	initial_joints_nao(1, 0) = joint_msg->joints[1];
	initial_joints_nao(2, 0) = joint_msg->joints[2];
	initial_joints_nao(3, 0) = joint_msg->joints[3];
	initial_joints_nao(4, 0) = joint_msg->joints[4];

	ROS_INFO("%lf %lf %lf %lf %lf", initial_joints_nao(0, 0), initial_joints_nao(1, 0), initial_joints_nao(2, 0), initial_joints_nao(3, 0), initial_joints_nao(4, 0));
	*/
	



	naoqi_bridge_msgs::JointAnglesWithSpeed msg;

	

	msg.joint_names = JointNames;
	msg.joint_angles = JointAngles;
    msg.speed = 0.5;

    pub.publish(msg);

    //std::cout << "Entrou" << std::endl;
}

void moveJoint::initialJoints()
{
	std::vector<std::string> JointNames;
	std::vector<float> JointAngles;




	JointNames.push_back("LWristYaw");
	JointAngles.push_back(0.0);

	JointNames.push_back("LElbowRoll");
	JointAngles.push_back(0.0);

	JointNames.push_back("LElbowYaw");
	JointAngles.push_back(0.0);

	JointNames.push_back("LShoulderRoll");
	JointAngles.push_back(0.0);

	float tmp10 = 0.0;

	JointNames.push_back("LShoulderPitch");
	JointAngles.push_back(tmp10);

		
	JointNames.push_back("RWristYaw");
	JointAngles.push_back(0.2);

	JointNames.push_back("RElbowRoll");
	JointAngles.push_back(0.0);

	JointNames.push_back("RElbowYaw");
	JointAngles.push_back(0.0);

	JointNames.push_back("RShoulderRoll");
	JointAngles.push_back(0.0);

	JointNames.push_back("RShoulderPitch");
	JointAngles.push_back(0.0);

	
	
	naoqi_bridge_msgs::JointAnglesWithSpeed msg;

	

	msg.joint_names = JointNames;
	msg.joint_angles = JointAngles;
    msg.speed = 0.5;

    pub.publish(msg);

    std::cout << "Move Joints" << std::endl;
}

void moveJoint::mainRoutine(int argc, char **argv)
{
	

	
	initialJoints();
   //( joint_angles_topic_, 10, &TeleopSubscriber::joint_angles_callback, this );
    ros::Subscriber sub = node.subscribe("move_joint", 1, &moveJoint::MoveJoints_CallBack, this);

    //pub = node.advertise<coco_nao::NaoJoint>("NAO_JOINTS", 1);




    ros::spin();
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "Client_Move_NAO");

	moveJoint NAO;

    

    NAO.mainRoutine(argc, argv);

	

	return 0;
}