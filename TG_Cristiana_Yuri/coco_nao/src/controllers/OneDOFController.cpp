#include "OneDOFController.hpp"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;


namespace DQ_robotics
{
	OneDOFController::OneDOFController(DQ_kinematics robot_armR, DQ_kinematics robot_armL, float translation_feedback_gain, double translation_damping)
	{
		
		//Initialization of argument parameters
	    robot_dofs_         = (robot_armR.links() - robot_armR.n_dummy());

	    robot_R_              = robot_armR;

	    robot_dofs_L_         = (robot_armL.links() - robot_armL.n_dummy());

	    robot_L_              = robot_armL;

	    kp_                  = translation_feedback_gain;

	    translation_damping_ = MatrixXd(1,1);
		
		translation_damping_(0,0) = translation_damping;

	    //Initilization of remaining parameters
	    joints_R_         = MatrixXd(robot_dofs_,1);
	    delta_joints_R_   = MatrixXd::Zero(robot_dofs_,1);

	    joints_L_         = MatrixXd(robot_dofs_,1);
	    delta_joints_L_   = MatrixXd::Zero(robot_dofs_,1);

	    joints_Aug_ 	  = MatrixXd(robot_dofs_*2,1);
	    delta_joints_Aug_   = MatrixXd::Zero(robot_dofs_*2,1);

	    j_meanR = VectorXd(5);

	    j_meanR(0) = 0.0;
	    j_meanR(1) = -0.50615;
	    j_meanR(2) = 0.0;
	    j_meanR(3) = 0.78975;
	    j_meanR(4) = 0.0;

	    j_meanL = VectorXd(5);

	    j_meanL(0) = 0.0;
	    j_meanL(1) = 0.50615;
	    j_meanL(2) = 0.0;
	    j_meanL(3) = -0.78975;
	    j_meanL(4) = 0.0;

	    //jacobians
	    analytical_jacobian_R_  		= MatrixXd(8,robot_dofs_);
	    analytical_jacobian_L_  		= MatrixXd(8,robot_dofs_);
	    relative_jacobian_				= MatrixXd(8,robot_dofs_*2);
	    relative_jacobian_translation	= MatrixXd(4,robot_dofs_*2);
	    relative_jacobian_distance		= MatrixXd(1,robot_dofs_*2);
	    
		//translation_jacobian_ = MatrixXd(4,robot_dofs_);
		distance_jacobian_pseudoinverse_   = MatrixXd(robot_dofs_*2,1);
		s_jacobian_R = MatrixXd(1, robot_dofs_);
		s_jacobian_L = MatrixXd(1, robot_dofs_);
		s_jacobian_Aug = MatrixXd(1, robot_dofs_*2);
		pseudo_inv_null_damped = MatrixXd(robot_dofs_, 1);

		//nullspace
		nullspace_projector_  = MatrixXd(robot_dofs_, robot_dofs_);
		nullspace = VectorXd(robot_dofs_);


	    identity4_           = Matrix<double,4,4>::Identity();
		identityDOFS_        = MatrixXd::Identity(robot_dofs_,robot_dofs_);
		identityAug_		 = MatrixXd::Identity(robot_dofs_*2,robot_dofs_*2);
		
		//error
		epsilon = 0.02;

		s_R_ = 0.0;
   		s_bar_R = 0.0;
   		s_L_ = 0.0;
   		s_bar_L = 0.0;
		GAIN_S = 1;

	    error_distance_  = VectorXd(1);

	    relative_pose_ 			  = DQ(0,0,0,0,0,0,0,0);
	    relative_pose_2_ 			  = DQ(0,0,0,0,0,0,0,0);
	    absolute_pose_ 			  = DQ(0,0,0,0,0,0,0,0);
	    current_pose_R_           = DQ(0,0,0,0,0,0,0,0);
	    current_pose_L_           = DQ(0,0,0,0,0,0,0,0);
		//reference_translation_       = DQ(0,0,0,0,0,0,0,0);
	    distance = 0;
	}

	VectorXd OneDOFController::getNewJointPositions( const DQ obj_pose, const VectorXd jointsAug)
	{

	    delta_joints_Aug_ = getNewJointVelocities(obj_pose, jointsAug);

	    // Send updated thetas to simulation
	    return (joints_Aug_ + delta_joints_Aug_);

	}

	VectorXd OneDOFController::getNewJointVelocities( const DQ obj_pose, const VectorXd jointsAug)
	{
		std::cout << "Entroy " << std::endl;
	    
	    ///--Remapping arguments
	    MatrixXd jointsR(5,1);
	    MatrixXd jointsL(5,1);

	   
	    joints_Aug_ = jointsAug;
	

	    for(int i = 0; i<5; i++)
        {
            jointsL(i, 0) = jointsAug(i);
            jointsR(i, 0) = jointsAug(i+5);
        }
        joints_L_=jointsL;
        joints_R_=jointsR;



		//Get translation and rotation individually
		distance = 0.40;

	    ///--Controller Step
	     
	    
	    //Calculate jacobians and FKM
	    current_pose_R_   = robot_R_.fkm(joints_R_);
	    current_pose_L_   = robot_L_.fkm(joints_L_);

	    //std::cout << "fkm controller: " << current_pose_R_ << std::endl;

	    
	    //Calculate Relative Pose

	   	relative_pose_ = current_pose_R_.conj()*current_pose_L_;
	   	//std::cout << "relative pose " << std::endl;

	   	//Calculate (x_rel)^1/2
	 
	    //std::cout << "absolute pose " << std::endl;

	    //Jacobians

	    analytical_jacobian_R_  = robot_R_.analyticalJacobian(joints_R_);
	    analytical_jacobian_L_  = robot_L_.analyticalJacobian(joints_L_);

	    MatrixXd relativeJ_tmp1, relativeJ_tmp2;



	    relativeJ_tmp1 = Hplus8(current_pose_R_.conj())*analytical_jacobian_L_;
	    relativeJ_tmp2 = Hminus8(current_pose_L_)*C8()*analytical_jacobian_R_;

	    MatrixXd rel(relativeJ_tmp1.rows(), relativeJ_tmp1.cols()+relativeJ_tmp2.cols());

	    rel   	<< relativeJ_tmp1, relativeJ_tmp2;
	    relative_jacobian_ = rel;

	    relative_jacobian_translation = jacobp(relative_jacobian_, vec8(relative_pose_));
	 	relative_jacobian_distance	=  2*(vec4(relative_pose_.translation())).transpose()*relative_jacobian_translation;	

	 	relative_jacobian_distance(0, 4) = 0.0;
		relative_jacobian_distance(0, 9) = 0.0;


	    //Error

	    error_distance_(0) = (distance -  vec4(relative_pose_.translation() ).norm());
	    //std::cout << "Erro " << error_distance_ << std::endl;

	    //Calculate pseudoinverse

		distance_jacobian_pseudoinverse_ = pseudoInverse(relative_jacobian_distance);

		
		//std::cout << "pseudoinverse " << std::endl;
						 
		float tmp = 0;
		s_R_ = 0;
		s_L_ = 0;
		s_bar_R = 0;
		s_bar_L = 0;

		//Nullspace projector

		
		for(int i = 0; i < robot_dofs_; i++)
		{
			
			tmp = joints_R_(i) - j_meanR(i);
			s_R_= s_R_ + 1/2 * tmp*tmp;
			s_bar_R = s_bar_R + tmp;
			
			s_jacobian_R(0, i) = tmp;

		}

		for(int i = 0; i < robot_dofs_; i++)
		{
			
			tmp = joints_L_(i) - j_meanL(i);
			s_L_ = s_L_ + 1/2 * tmp*tmp;
			s_bar_L = s_bar_L + tmp;
			
			s_jacobian_L(0, i) = tmp;

		}
		//std::cout << "s_ " << std::endl;


   		s_jacobian_Aug << s_jacobian_L, s_jacobian_R;

        MatrixXd s_Aug;

        s_Aug = MatrixXd(1,1);

       
        MatrixXd tmp_JsPi , tmp_PInv;

        s_Aug(0,0) = s_R_ + s_L_;
        
        std::cout << "soma " << std::endl;

        MatrixXd projectorTmp = (distance_jacobian_pseudoinverse_)*relative_jacobian_distance ;

        std::cout << "ProjTmp " << std::endl;

		nullspace_projector_ = identityAug_ - projectorTmp;
		

		std::cout << "pi: " << nullspace_projector_ << std::endl;

		pseudo_inv_null_damped = (s_jacobian_Aug*nullspace_projector_).transpose();

		
		tmp_JsPi = (s_jacobian_Aug*nullspace_projector_);
		
		tmp_PInv = tmp_JsPi*tmp_JsPi.transpose() + translation_damping_;
		
		pseudo_inv_null_damped = pseudo_inv_null_damped*(tmp_PInv).inverse();
		
		nullspace = (-1)*(pseudo_inv_null_damped * (GAIN_S*s_Aug - s_jacobian_Aug*distance_jacobian_pseudoinverse_*kp_*error_distance_));

		std::cout << "nullspace: " << nullspace << std::endl;

		
	    delta_joints_Aug_ = distance_jacobian_pseudoinverse_ * kp_ * error_distance_+ 0.1*kp_*nullspace_projector_*nullspace;

	    std::cout << "delta_joints_: " << delta_joints_Aug_ << std::endl;

	    return delta_joints_Aug_;
	}

}



    
    
