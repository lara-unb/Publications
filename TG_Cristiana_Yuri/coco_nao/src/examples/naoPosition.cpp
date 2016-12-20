//includes ROS




//includes DHNAO e controllers
#include "../controllers/NaoTestController.hpp"
#include "../robot_dh/NAO.h" //Has NAOKinemetics_LArm()
#include "naoPosition.hpp"


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
    cout << "Teste" << endl;
    fileIndexPos = Save.createFile("/home/cris/catkin_ws/src/coco_nao/jointsDQ_oneArmTranslation_left_6.txt");
    fileIndexError = Save.createFile("/home/cris/catkin_ws/src/coco_nao/error_oneArmTranslation_left_6.txt");
    fileIndexInitial = Save.createFile("/home/cris/catkin_ws/src/coco_nao/initialjoints_oneArmTranslation_left_6.txt");
    fileIndexFinal = Save.createFile("/home/cris/catkin_ws/src/coco_nao/joints_oneArmTranslation_left_6.txt");
    fileIndexJacob = Save.createFile("/home/cris/catkin_ws/src/coco_nao/jacobian_oneArmTranslation_left_6.txt");
    fileIndexDelta = Save.createFile("/home/cris/catkin_ws/src/coco_nao/delta_oneArmTranslation_left_6.txt");

    initialTime = ros::Time::now();

    Save.savePosition(DQ(1.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.03, 0.05), 0.0, fileIndexPos);
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
	Matrix<double,5,1> initial_joints_nao;
	
	initial_joints_nao(0, 0) = joint_msg->joints[0]; 
	initial_joints_nao(1, 0) = joint_msg->joints[1];
	initial_joints_nao(2, 0) = joint_msg->joints[2];
	initial_joints_nao(3, 0) = joint_msg->joints[3];
	initial_joints_nao(4, 0) = joint_msg->joints[4];


	ROS_INFO("Joints %lf %lf %lf %lf %lf", initial_joints_nao(0, 0), initial_joints_nao(1, 0), initial_joints_nao(2, 0), initial_joints_nao(3, 0), initial_joints_nao(4, 0));
    
    //*****************
    //Matrix<double,6,1> eff_pose_reference_nao(req.eff_pose_reference.data());

     cout << "NAO TEST MUDOU" << endl;
   
    const double pi2 = M_PI_2;

    //Initial Joint Values
    Matrix<double,5,1> joints;
    
    joints << initial_joints_nao;
    
    //ROS_INFO("joints: %lf %lf %lf %lf %lf", joints(0, 0), joints(1, 0), joints(2, 0), joints(3, 0), joints(4, 0));
    
    //Gain 
    float kp = 0.5;
 	double t_damp = 0.001;

    //receiving from DH

    cout << "AQUI " << endl;

    DQ_kinematics nao_chain = NAOKinematics_LArm();

    //cout << "base: " << nao_chain.base() << endl;

    // Get Time

    ros::Time timeNAO = ros::Time::now();

    timeNAO.sec -= initialTime.sec;

    double currentTime = (timeNAO.nsec/1000000 + timeNAO.sec*1000);

    
    if(contadorVergonha == 0)
    {
        //cout << "oi da contadorVergonha" << endl;
        DQ eff_pose_initial = nao_chain.fkm(joints);

    
        //DQ transEff(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05, 0.0);
        //DQ transEff(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5, 0.5);



        //obj_pose = eff_pose_initial*transEff;

        //obj_pose = transEff; //Absolute Pose


        contadorVergonha++;
    }
   
    
    TranslationController controller(nao_chain, kp, t_damp); //controller constructor


    //Control Loop Variables Inicialization
    DQ eff_pose_current(0);
	DQ eff_pose_older = nao_chain.fkm(joints);

    cout << "fkm: " << eff_pose_older << endl;
    cout << "translation: " << eff_pose_older.translation() << endl;
	
    double translation_threshold = 0.001; // Small to go on the loop

    
    //Control Loop

	eff_pose_current = nao_chain.fkm(joints);
    //obj_pose = (eff_pose_current.P()).conj()*DQ(1.0, 0.0, 0.0, 0.0, 0.0, 0.057, -0.015, 0.02)*eff_pose_current.P();
    obj_pose = DQ(1.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.03, 0.05);
    
    
	//Difference Between the end effector position and the object
	eff_translation_difference = vec4( obj_pose.translation() - eff_pose_current.translation()); 
    DQ err_save = eff_translation_difference;
    cout << "translations diff " << err_save.vec4() << endl;

    
    if( (eff_translation_difference.vec4().norm() > translation_threshold) )
    {   
         
        //One controller step
        joints = controller.getNewJointPositions(obj_pose, joints);
        cout << "Joints " << joints << endl;
     
        //End of control check
        eff_pose_current = nao_chain.fkm(joints);

		// Get translation differenceS
		eff_translation_difference = vec4( obj_pose.translation() - eff_pose_current.translation()); 
        //cout << "translations diff " << eff_translation_difference.vec4().norm() << endl;
    }
    
    //saveData(eff_pose_current, currentTime);
    MatrixXd Jacob = controller.translation_jacobian_;
    VectorXd delta_joints = controller.delta_joints_;

    Save.saveJacob(Jacob, currentTime, fileIndexJacob);
    Save.saveJoints(delta_joints.transpose(), currentTime, fileIndexDelta);
    Save.savePosition(eff_pose_older, currentTime, fileIndexPos);
    Save.saveError(vec4(err_save), currentTime, fileIndexError);
    Save.saveJoints(initial_joints_nao.transpose(), currentTime, fileIndexInitial);
    Save.saveJoints(joints.transpose(), currentTime, fileIndexFinal);
    Vector4d end_eff_position = vec4(eff_pose_current.translation());

    //*******
	coco_nao::MoveJoint msg;

    msg.chain = Chain;

    vector<double> aux;
    
    aux.push_back(joints(0,0));
    aux.push_back(joints(1,0));
    aux.push_back(joints(2,0));
    aux.push_back(joints(3,0));
    aux.push_back(joints(4,0));

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


