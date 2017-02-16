//includes ROS




//includes DHNAO e controllers
#include "../controllers/FixedPositionController.hpp"
#include "../robot_dh/NAO.h" //Has NAOKinemetics_LArm()
#include "NAOrelative.hpp"


#include <cmath> //For fabs and M_PI_2

using namespace Eigen;
using namespace DQ_robotics;
using namespace std;

/****************************************************************************************************/
//																									//
//             		 This function provides the service for getting the joints,						//
//     it takes in the request and response type defined in the srv file and returns a boolean.		//
//																									//
//***************************************************************************************************/



cocoNAO::cocoNAO()
{

	pub = nodeHandle.advertise<coco_nao::MoveJoint>("move_joint", 1);
    contadorVergonha = 0;
    eff_translation_difference = DQ(20);

    delta_joints(0, 0) = 0.0; 
    delta_joints(1, 0) = 0.0;
    delta_joints(2, 0) = 0.0;
    delta_joints(3, 0) = 0.0;
    delta_joints(4, 0) = 0.0;

    //Results.open("/home/cris/catkin_ws/src/coco_nao/resultados_translation_controller1.txt");

    fileIndexPosL = Save.createFile("/home/cris/catkin_ws/src/coco_nao/joints_RelController_left_1.txt");
    fileIndexPosR = Save.createFile("/home/cris/catkin_ws/src/coco_nao/joints_RelController_right_1.txt");
    fileIndexError = Save.createFile("/home/cris/catkin_ws/src/coco_nao/error_RelController_1.txt");
    fileIndexPosRel = Save.createFile("/home/cris/catkin_ws/src/coco_nao/joints_RelController_Rel_1.txt");

    fileIndexJacob = Save.createFile("/home/cris/catkin_ws/src/coco_nao/jacobian_RelCotroller_1.txt");
    fileIndexDelta = Save.createFile("/home/cris/catkin_ws/src/coco_nao/delta_RelCotroller_1.txt");

    fileIndexInitial = Save.createFile("/home/cris/catkin_ws/src/coco_nao/initialjoints_RelCotroller_1.txt");
    fileIndexFinal = Save.createFile("/home/cris/catkin_ws/src/coco_nao/joints_RelCotroller_1.txt");
    

    DQ initial = (DQ(1.0, 0.0, 0.0, 0.0, 0.0, -0.015, 0.15, 0.0));
    Save.savePosition(initial, 0.0, fileIndexPosRel);

    initialTime = ros::Time::now();
}

cocoNAO::~cocoNAO()
{
    Results.close();
}

void cocoNAO::DQinvKin(const coco_nao::NaoJointPtr& joint_msg)
{
	

	//receiving from service (NAO angles)
	string Chain = joint_msg->chain;
	//cout << Chain << endl; 
	Matrix<double,10,1> initial_joints_nao;
	
	initial_joints_nao(0, 0) = joint_msg->joints[0]; 
	initial_joints_nao(1, 0) = joint_msg->joints[1];
	initial_joints_nao(2, 0) = joint_msg->joints[2];
	initial_joints_nao(3, 0) = joint_msg->joints[3];
	initial_joints_nao(4, 0) = joint_msg->joints[4];
    initial_joints_nao(5, 0) = joint_msg->joints[5]; 
    initial_joints_nao(6, 0) = joint_msg->joints[6];
    initial_joints_nao(7, 0) = joint_msg->joints[7];
    initial_joints_nao(8, 0) = joint_msg->joints[8];
    initial_joints_nao(9, 0) = joint_msg->joints[9];


	ROS_INFO("%lf %lf %lf %lf %lf", initial_joints_nao(0, 0), initial_joints_nao(1, 0), initial_joints_nao(2, 0), initial_joints_nao(3, 0), initial_joints_nao(4, 0));
    ROS_INFO("%lf %lf %lf %lf %lf", initial_joints_nao(5, 0), initial_joints_nao(6, 0), initial_joints_nao(7, 0), initial_joints_nao(8, 0), initial_joints_nao(9, 0));
    
    
    //*****************
    //Matrix<double,6,1> eff_pose_reference_nao(req.eff_pose_reference.data());


   
    const double pi2 = M_PI_2;

    

    //Initial Joint Values
    Matrix<double,10,1> joints;
    
    joints << initial_joints_nao;

    VectorXd jointsTmpL(5);
    VectorXd jointsTmpR(5);

    for(int i = 0; i<5; i++)
    {
        jointsTmpL(i) = joints(i,0);
        jointsTmpR(i) = joints(i+5,0);
    }
    
    //ROS_INFO("joints: %lf %lf %lf %lf %lf", joints(0, 0), joints(1, 0), joints(2, 0), joints(3, 0), joints(4, 0));
    
    //Gain 
    float kp = 0.5;
 	double t_damp = 0.001;

    //receiving from DH



    DQ_kinematics nao_chainL = NAOKinematics_LArm();
    DQ_kinematics nao_chainR = NAOKinematics_RArm();

    //cout << "base: " << nao_chain.base() << endl;

    // Get Time

    ros::Time timeNAO = ros::Time::now();

    timeNAO.sec -= initialTime.sec;

    double currentTime = (timeNAO.nsec/1000000 + timeNAO.sec*1000);

    DQ eff_pose_initialL = nao_chainL.fkm(jointsTmpL);
    DQ eff_pose_initialR = nao_chainR.fkm(jointsTmpR);
    


    RelativeController controller(nao_chainR, nao_chainL, kp, t_damp); //controller constructor


    //Control Loop Variables Inicialization
    DQ eff_pose_currentL(0);
    DQ eff_pose_currentR(0);


   

	DQ eff_pose_olderL = nao_chainL.fkm(jointsTmpL);
    DQ eff_pose_olderR = nao_chainR.fkm(jointsTmpR);

	
    double translation_threshold = 0.001; // Small to go on the loop

    
    //Control Loop

	eff_pose_currentL = nao_chainL.fkm(jointsTmpL);
    eff_pose_currentR = nao_chainR.fkm(jointsTmpR);

    DQ transEff = (eff_pose_currentR.P()).conj()*DQ(1.0, 0.0, 0.0, 0.0, 0.0, -0.015, 0.15, 0.0)*eff_pose_currentR.P();
    transEff = transEff*(transEff.norm()).inv();

    DQ x_rel;

    x_rel = eff_pose_currentR.conj()*eff_pose_currentL;

	//Difference Between the end effector position and the object

	eff_translation_difference = vec4( transEff.translation() - x_rel.translation()); 
     DQ err_save = eff_pose_currentR.P()*(transEff.translation() - x_rel.translation())*(eff_pose_currentR.P()).conj();
    cout << "TransEff " <<transEff.translation() << endl << "x_rel " << ((eff_pose_currentR.P())*x_rel.translation())*eff_pose_currentR.P().conj() << endl << vec4(err_save) << endl;

    
    if( (eff_translation_difference.vec4().norm() > translation_threshold) )
    {   
         
        //One controller step
        joints = controller.getNewJointPositions(transEff, joints);

        for(int i = 0; i<5; i++)
        {
            jointsTmpL(i) = joints(i,0);
            jointsTmpR(i) = joints(i+5,0);
        }
     
        //End of control check
        eff_pose_currentL = nao_chainL.fkm(jointsTmpL);

        eff_pose_currentR = nao_chainR.fkm(jointsTmpR);
        x_rel = eff_pose_currentR.conj()*eff_pose_currentL;

		// Get translation differenceS
		eff_translation_difference = vec4( transEff.translation() - x_rel.translation()); 
        //cout << "translations diff " << eff_translation_difference.vec4().norm() << endl;
    }
    
    MatrixXd Jacob = controller.relative_jacobian_translation;
    VectorXd delta_joints = controller.delta_joints_Aug_;

    Save.saveJacob(Jacob, currentTime, fileIndexJacob);
    Save.saveJoints(delta_joints.transpose(), currentTime, fileIndexDelta);
    Save.saveJoints(initial_joints_nao.transpose(), currentTime, fileIndexInitial);
    Save.saveJoints(joints.transpose(), currentTime, fileIndexFinal);

    Save.savePosition(eff_pose_olderR, currentTime, fileIndexPosR);
    Save.saveError(vec4(err_save), currentTime, fileIndexError);

    Save.savePosition(eff_pose_olderL, currentTime, fileIndexPosL);
    Save.savePosition(x_rel, currentTime, fileIndexPosRel);
    //saveData(eff_pose_currentL, currentTime);
    
    //Vector4d end_eff_position = vec4(eff_pose_currentL.translation());

    //*******
	coco_nao::MoveJoint msg;

    msg.chain = Chain;

    vector<double> aux;
    
    aux.push_back(joints(0,0));
    aux.push_back(joints(1,0));
    aux.push_back(joints(2,0));
    aux.push_back(joints(3,0));
    aux.push_back(joints(4,0));
    aux.push_back(joints(5,0));
    aux.push_back(joints(6,0));
    aux.push_back(joints(7,0));
    aux.push_back(joints(8,0));
    aux.push_back(joints(9,0));


    //ROS_INFO("joints: %lf %lf %lf %lf %lf", aux[0], aux[1], aux[2], aux[3], aux[4]);

    msg.new_joints = aux;
    pub.publish(msg);
    //pub.thetas.assign(aux, aux+5);     
    

}

void cocoNAO::saveData(DQ pose, double time)
{
    Results << pose << ", " << time << endl;
}

/****************************************************************************************************/
//																									//
//             				 Starting the thread and the Services									//
//																									//
//***************************************************************************************************/

int main(int argc, char **argv)
{
	//ros stuff
	
	ros::init(argc, argv, "Test_Controller_For_NAO_Left_arm");
	ros::NodeHandle nodeHandleMain;
	ROS_INFO("Running Test Controller...");

	cocoNAO NAO;

	//ros::ServiceServer service = nodeHandleMain.advertiseService("Getting_IMU", DQinvKin );

	ros::Subscriber subscriber = nodeHandleMain.subscribe("NAO_JOINTS", 1, &cocoNAO::DQinvKin, &NAO);

	ROS_INFO("Ready To Start Controll Loop...");

	ros::spin(); //Will not return until the node has been shutdown, either through a call to ros::shutdown() or a Ctrl-C.

	return 0;

}


