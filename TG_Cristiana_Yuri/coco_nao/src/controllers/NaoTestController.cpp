#include "NaoTestController.hpp"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;


namespace DQ_robotics
{
	TranslationController::TranslationController(DQ_kinematics robot, float translation_feedback_gain, double translation_damping)
	{
		
		//Initialization of argument parameters
	    robot_dofs_          = (robot.links() - robot.n_dummy());

	    robot_               = robot;

	    kp_                  = translation_feedback_gain;

	    translation_damping_ = MatrixXd(1,1);
		
		translation_damping_(0,0) = translation_damping;

	    //Initilization of remaining parameters
	    joints_         = MatrixXd(robot_dofs_,1);
	    delta_joints_   = MatrixXd::Zero(robot_dofs_,1);

	    j_mean = VectorXd(5);

	    j_mean(0) = 0.0;
	    j_mean(1) = 0.50615;
	    j_mean(2) = 0.0;
	    j_mean(3) = -0.78975;
	    j_mean(4) = 0.0;

	    //jacobians
	    analytical_jacobian_  = MatrixXd(8,robot_dofs_);
		translation_jacobian_ = MatrixXd(4,robot_dofs_);
		translation_jacobian_pseudoinverse_   = MatrixXd(robot_dofs_,4);
		s_jacobian = MatrixXd(1, robot_dofs_);
		pseudo_inv_null_damped = MatrixXd(robot_dofs_, 1);

		//nullspace
		nullspace_projector_  = MatrixXd(robot_dofs_, robot_dofs_);
		nullspace = VectorXd(robot_dofs_);


	    identity4_           = Matrix<double,4,4>::Identity();
		identityDOFS_        = MatrixXd::Identity(robot_dofs_,robot_dofs_);

		//error
		epsilon = 0.02;

		s_ = 0.0;
   		s_bar = 0.0;
		GAIN_S = 1;

	    error_translation_  = MatrixXd(4,1);

	    
	    current_pose_           = DQ(0,0,0,0,0,0,0,0);
		reference_translation_       = DQ(0,0,0,0,0,0,0,0);

		

	}

	VectorXd TranslationController::getNewJointPositions( const DQ obj_pose, const VectorXd joints)
	{

	    delta_joints_ = getNewJointVelocities(obj_pose, joints);

	    // Send updated thetas to simulation
	    return (joints_ + delta_joints_);

	}

	VectorXd TranslationController::getNewJointVelocities( const DQ obj_pose, const VectorXd joints)
	{
		
		
	    ///--Remapping arguments
	    joints_ = joints;

		//Get translation and rotation individually
		reference_translation_ = obj_pose.translation();

	    ///--Controller Step
	     
	    
	    //Calculate jacobians and FKM
	    current_pose_    = robot_.fkm(joints_);

	    //std::cout << "fkm controller: " << current_pose_ << std::endl;


	    analytical_jacobian_  = robot_.analyticalJacobian(joints_);
	    translation_jacobian_ = translationJacobian(analytical_jacobian_, vec8(current_pose_));

	   




	    //std::cout << "translation jacobian: " << translation_jacobian_ << std::endl;


	    //Error
	    error_translation_ = vec4( reference_translation_ -  current_pose_.translation() );


	   	std::cout << "errortranslation: " << error_translation_ << std::endl;
		//Pseudoinverses calculation 
		//JT'(JT*JT' + λI)^-1		
	    MatrixXd translation_jacobian_transpose, translation_jacobian_tmp;

		translation_jacobian_transpose = (translation_jacobian_.transpose());
		
		translation_jacobian_tmp = translation_jacobian_* translation_jacobian_transpose + translation_damping_(0,0)*identity4_;
		
		translation_jacobian_pseudoinverse_	= translation_jacobian_transpose*translation_jacobian_tmp.inverse();

		//translation_jacobian_pseudoinverse_ = pseudoInverse(translation_jacobian_);

		//std::cout << "pseudo inversa: " << translation_jacobian_pseudoinverse_ << std::endl;

						 
		float tmp = 0;
		//Nullspace projector

		s_ = 0;
		s_bar = 0;

		for(int i = 0; i < robot_dofs_; i++)
		{
			
			tmp = joints_(i) - j_mean(i);
			s_ = s_ + 1/2 * tmp*tmp;
			s_bar = s_bar + tmp;
			
			s_jacobian(0, i) = tmp;

		}
   
        MatrixXd s_Matrix;
        
        s_Matrix = MatrixXd(1,1);
        MatrixXd tmp_JsPi , tmp_PInv;

        s_Matrix(0,0) = s_;
        
        
		nullspace_projector_ = (identityDOFS_ - (translation_jacobian_pseudoinverse_)*translation_jacobian_ );
		
		//std::cout << "pi: " << nullspace_projector_ << std::endl;

		pseudo_inv_null_damped = (s_jacobian*nullspace_projector_).transpose();
		
		tmp_JsPi = (s_jacobian*nullspace_projector_);
		
		tmp_PInv = tmp_JsPi*tmp_JsPi.transpose() + translation_damping_;
		
		pseudo_inv_null_damped = pseudo_inv_null_damped*(tmp_PInv).inverse();
		
		nullspace = (-1)*(pseudo_inv_null_damped * (GAIN_S*s_Matrix - s_jacobian*translation_jacobian_pseudoinverse_*kp_*error_translation_));

		//std::cout << "nullspace: " << nullspace << std::endl;

		
	    delta_joints_ = translation_jacobian_pseudoinverse_ * kp_ * error_translation_+ kp_*0.1*nullspace_projector_*nullspace;

	    

	    //std::cout << "delta_joints_: " << delta_joints_ << std::endl;

	    return delta_joints_;

	}



}


/*

%SET JOINTS TO ZERO
SetJoints(clientID, vrep, handles.joints, [0 0 0 0 0]);
K = 0.5;
lambda = 0.003
*/

    
    
