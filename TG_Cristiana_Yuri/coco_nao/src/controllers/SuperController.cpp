#include "SuperController.hpp"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

#define MATRIX_DIMENTION 13


namespace DQ_robotics
{
	MatrixXd SuperController::integral_error_     = MatrixXd::Zero(MATRIX_DIMENTION,1);
	SuperController::SuperController(DQ_kinematics robot_armR, DQ_kinematics robot_armL, MatrixXd translation_feedback_gain, double translation_damping)
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
	    rotation_jacobian_R_			= MatrixXd(4,robot_dofs_*2);
	    rotation_jacobian_L_			= MatrixXd(4,robot_dofs_*2);

	    relative_jacobian_				= MatrixXd(8,robot_dofs_*2);
	    relative_jacobian_translation	= MatrixXd(4,robot_dofs_*2);
	    relative_jacobian_2				= MatrixXd(8,robot_dofs_*2);
	    ext_jacobian_x2					= MatrixXd(8,robot_dofs_*2);
	    absolute_jacobian 				= MatrixXd(8,robot_dofs_*2);
	    absolute_jacobian_translation	= MatrixXd(4,robot_dofs_*2);

	    relative_jacobian_distance		= MatrixXd(1,robot_dofs_*2);

		main_jacobian_   				= MatrixXd(MATRIX_DIMENTION,robot_dofs_*2);
		main_jacobian_pseudoinverse_   	= MatrixXd(robot_dofs_*2,MATRIX_DIMENTION);

		s_jacobian_R = MatrixXd(1, robot_dofs_);
		s_jacobian_L = MatrixXd(1, robot_dofs_);
		s_jacobian_Aug = MatrixXd(1, robot_dofs_*2);
		pseudo_inv_null_damped = MatrixXd(robot_dofs_, 1);

		//nullspace
		nullspace_projector_  = MatrixXd(robot_dofs_, robot_dofs_);
		nullspace = VectorXd(robot_dofs_);


	    identity4_           = Matrix<double,4,4>::Identity();
		identityDOFS_        = MatrixXd::Identity(robot_dofs_,robot_dofs_);
		identityAug_		 = MatrixXd::Identity(MATRIX_DIMENTION,MATRIX_DIMENTION);
		
		//error
		epsilon = 0.02;

		s_R_ = 0.0;
   		s_bar_R = 0.0;
   		s_L_ = 0.0;
   		s_bar_L = 0.0;
		GAIN_S = 1;

	    error_rotation_R_  = MatrixXd(4,1);
	    error_rotation_L_  = MatrixXd(4,1);
	    error_absolute_  = MatrixXd(4,1);
	    error_distance_  = MatrixXd(1,1);
	    error_ 			   = MatrixXd(MATRIX_DIMENTION,1);

	    relative_pose_ 			  = DQ(0,0,0,0,0,0,0,0);
	    relative_pose_2_ 			  = DQ(0,0,0,0,0,0,0,0);
	    absolute_pose_ 			  = DQ(0,0,0,0,0,0,0,0);
	    current_pose_R_         = DQ(0,0,0,0,0,0,0,0);
	    current_pose_L_         = DQ(0,0,0,0,0,0,0,0);
		reference_rotation_R_   = DQ(0,0,0,0,0,0,0,0);
		reference_rotation_L_   = DQ(0,0,0,0,0,0,0,0);

	}

	VectorXd SuperController::getNewJointPositions( const DQ obj_pose, const VectorXd jointsAug)
	{

	    delta_joints_Aug_ = getNewJointVelocities(obj_pose, jointsAug);

	    // Send updated thetas to simulation
	    return (joints_Aug_ + delta_joints_Aug_);

	}

	VectorXd SuperController::getNewJointVelocities( const DQ obj_pose, const VectorXd jointsAug)
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
		//Reference rotation to the right arm is a 90º rotation around the x axis
		DQ tmp_t(0.0, 0.0, 0.0, 0.0);
		DQ tmp_r(cos(M_PI_2/2), sin(M_PI_2/2), 0.0, 0.0);
		DQ tmp_q = tmp_r + 0.5*E_*tmp_t*tmp_r;
		reference_rotation_R_ = tmp_q;

		//Reference rotation to the left arm is a -90º rotation around the x axis
		tmp_r = DQ(cos(-M_PI_2/2), sin(-M_PI_2/2), 0.0, 0.0);
		tmp_q = tmp_r + 0.5*E_*tmp_t*tmp_r;
		reference_rotation_L_ =tmp_q;

	    ///--Controller Step
	     
	    //Calculate jacobians and FKM
	    current_pose_R_   = robot_R_.fkm(joints_R_);
	    current_pose_L_   = robot_L_.fkm(joints_L_);

	    relative_pose_ = current_pose_R_.conj()*current_pose_L_;
	    relative_pose_2_ =  exp(0.5*log(relative_pose_));
	    absolute_pose_ = current_pose_R_*relative_pose_2_;
	    //std::cout << "fkm controller: " << current_pose_R_ << std::endl;

	    //Jacobians

	    analytical_jacobian_R_  = robot_R_.analyticalJacobian(joints_R_);
	    analytical_jacobian_L_  = robot_L_.analyticalJacobian(joints_L_);

	    rotation_jacobian_R_ = Hminus4(reference_rotation_R_)*C4()*analytical_jacobian_R_.block(0, 0, 4, 5);
	    rotation_jacobian_L_ = Hminus4(reference_rotation_L_)*C4()*analytical_jacobian_L_.block(0, 0, 4, 5);

	    MatrixXd relativeJ_tmp1, relativeJ_tmp2;

	    relativeJ_tmp1 = Hplus8(current_pose_R_.conj())*analytical_jacobian_L_;
	    relativeJ_tmp2 = Hminus8(current_pose_L_)*C8()*analytical_jacobian_R_;

	    MatrixXd rel(relativeJ_tmp1.rows(), relativeJ_tmp1.cols()+relativeJ_tmp2.cols());

	    rel   	<< relativeJ_tmp1, relativeJ_tmp2;
	    relative_jacobian_ = rel;

	    relative_jacobian_translation = jacobp(relative_jacobian_, vec8(relative_pose_));

	    //Distance Jacobian

	 	relative_jacobian_distance	=  2*(vec4(relative_pose_.translation())).transpose()*relative_jacobian_translation;	

	    //std::cout << "relative ok " << std::endl;
    	//Jacobiana Absoluta
    
    	MatrixXd JRel2_tmp1, JRel2_tmp2, relative_jacobian_14;

    	relative_jacobian_14 = MatrixXd(4,robot_dofs_*2);

    	for(int i = 0; i < 4; i++)
    	{
    		for(int j = 0; j < 10; j++)
    		{
    			relative_jacobian_14 (i, j) = relative_jacobian_(i,j);
    		}
    	}

    	//std::cout << "dor abs " << std::endl;

    	JRel2_tmp1 = 0.5*( Hminus4((relative_pose_2_.P()).conj())*relative_jacobian_14);
    	//std::cout << "JRel1 " << std::endl;

    	JRel2_tmp2 = 0.25*( Hminus4(relative_pose_2_.P())*relative_jacobian_translation + Hplus4(relative_pose_.translation())*JRel2_tmp1);
    	//std::cout << "JRel2 " << std::endl;
    
    	relative_jacobian_2 << JRel2_tmp1, JRel2_tmp2;
    	//std::cout << "Concatenate " << relative_jacobian_2.rows() << std::endl;

	    ext_jacobian_x2 << MatrixXd::Zero(8, 5), analytical_jacobian_R_;
	    //std::cout << "ext jacob " << std::endl;
    
    	absolute_jacobian = Hminus8(relative_pose_2_)*ext_jacobian_x2 + hamiplus8(current_pose_R_)*relative_jacobian_2;

    	//std::cout << "abs " << std::endl;
    
    	absolute_jacobian_translation = jacobp(absolute_jacobian, vec8(absolute_pose_));

    	//std::cout << "abs trans " << std::endl;
   
	    /*for(int i = 0; i < 4; i++)
	    {
	    	absolute_jacobian_translation(i, 4)  = 0;
	    	absolute_jacobian_translation(i, 9) = 0;	    	
	    }*/


	    //std::cout << "relative ok " << std::endl;
    	//Jacobiana Aumentada
    	MatrixXd tmpL_J(4, robot_dofs_*2), tmpR_J(4, robot_dofs_*2);

    	tmpL_J << rotation_jacobian_L_, MatrixXd::Zero(4, robot_dofs_);
    	tmpR_J << MatrixXd::Zero(4, robot_dofs_), rotation_jacobian_R_;

    	main_jacobian_ << relative_jacobian_distance, tmpL_J, tmpR_J, absolute_jacobian_translation;
    	//main_jacobian_ << relative_jacobian_distance, absolute_jacobian_translation;

	    //Error

	    error_rotation_R_  = vec4(DQ(1) - ((current_pose_R_).conj()*reference_rotation_R_)).transpose();
	    error_rotation_L_  = vec4(DQ(1) - ((current_pose_L_).conj()*reference_rotation_L_)).transpose();

	    error_distance_(0, 0) = (distance -  vec4(relative_pose_.translation() ).norm());

	    error_absolute_ = vec4( obj_pose.translation() -  absolute_pose_.translation() );

	    error_ << error_distance_, error_rotation_L_, error_rotation_R_, error_absolute_;
	    integral_error_ = integral_error_ + error_;
	    //error_ << error_distance_, error_absolute_;
	    std::cout << "Erro " << error_ << std::endl;

	    //Calculate pseudoinverse
	    MatrixXd main_jacobian_transpose, main_jacobian_tmp;

		main_jacobian_transpose = (main_jacobian_.transpose());
		
		main_jacobian_tmp = main_jacobian_* main_jacobian_transpose + translation_damping_(0,0)*identityAug_;
		
		main_jacobian_pseudoinverse_	= main_jacobian_transpose*main_jacobian_tmp.inverse();

		//main_jacobian_pseudoinverse_ = pseudoInverse(main_jacobian_);
						 
		/*float tmp = 0;
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


   		s_jacobian_Aug << s_jacobian_L, s_jacobian_R;

        MatrixXd s_Aug;

        s_Aug = MatrixXd(1,1);

       
        MatrixXd tmp_JsPi , tmp_PInv;

        s_Aug(0,0) = s_R_ + s_L_;
        
        MatrixXd projectorTmp = (absolute_jacobian_pseudoinverse_)*absolute_jacobian_translation ;



		nullspace_projector_ = identityAug_ - projectorTmp;
		

		//std::cout << "pi: " << nullspace_projector_ << std::endl;

		pseudo_inv_null_damped = (s_jacobian_Aug*nullspace_projector_).transpose();

		
		tmp_JsPi = (s_jacobian_Aug*nullspace_projector_);
		
		tmp_PInv = tmp_JsPi*tmp_JsPi.transpose() + translation_damping_;
		
		pseudo_inv_null_damped = pseudo_inv_null_damped*(tmp_PInv).inverse();
		
		nullspace = (-1)*(pseudo_inv_null_damped * (GAIN_S*s_Aug - s_jacobian_Aug*absolute_jacobian_pseudoinverse_*kp_*error_absolute_));

		//std::cout << "nullspace: " << nullspace << std::endl;
*/
	    //delta_joints_Aug_ = main_jacobian_pseudoinverse_ * kp_ * error_;
	    float ki_ = 0.05;
	    delta_joints_Aug_ = main_jacobian_pseudoinverse_ * ( kp_*error_);

	    //std::cout << "ki_*integral_error_: " << integral_error_ << std::endl;

	    //std::cout << "delta_joints_: " << delta_joints_Aug_ << std::endl;

	    return delta_joints_Aug_;
	}

	void SuperController::setDistance(const float &distance) 
	{
		this->distance = distance;
	}
}



/*

%SET JOINTS TO ZERO
SetJoints(clientID, vrep, handles.joints, [0 0 0 0 0]);
K = 0.5;
lambda = 0.003
*/

    
    
