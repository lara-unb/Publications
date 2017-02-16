#pragma once

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../DQ_controller.h"
#include <Eigen/Dense>

using namespace Eigen;



namespace DQ_robotics
{
	class RotationController : public DQ_controller
	{
	public:

	private:
		DQ_kinematics robot_R_;
	    int robot_dofs_;
	    DQ_kinematics robot_L_;
	    int robot_dofs_L_;

		//double rotation_damping_;
		MatrixXd translation_damping_;

	    float kp_;

	    VectorXd j_meanR;
		VectorXd j_meanL; //media das juntas


	    VectorXd joints_R_; //joints
	    VectorXd delta_joints_R_; //variação das juntas

	    VectorXd joints_L_; //joints
	    VectorXd delta_joints_L_; //variação das juntas

	    VectorXd joints_Aug_;
	    


	    MatrixXd error_;

	    MatrixXd analytical_jacobian_R_;
	    MatrixXd analytical_jacobian_L_;
	    MatrixXd rotation_jacobian_R_;
	    MatrixXd rotation_jacobian_L_;
	    
	    MatrixXd main_jacobian_pseudoinverse_;

		MatrixXd nullspace_projector_;
		MatrixXd pseudo_inv_null_damped;
		VectorXd nullspace;

		MatrixXd identity4_;
		MatrixXd identity8_;
		MatrixXd identityDOFS_, identityAug_;

	    DQ current_pose_R_;
	    DQ current_pose_L_; 

		DQ reference_rotation_R_;
		DQ reference_rotation_L_;

		float epsilon;

		float s_R_;
		float s_bar_R;
		MatrixXd s_jacobian_R;
		MatrixXd s_jacobian_Aug;

		float s_L_;
		float s_bar_L;
		MatrixXd s_jacobian_L;

		int GAIN_S;

	public: //methods
		VectorXd error_rotation_R_;
	    VectorXd error_rotation_L_;
		MatrixXd main_jacobian_;
		VectorXd delta_joints_Aug_;
		
	    RotationController(DQ_kinematics robot_armR, DQ_kinematics robot_armL, float translation_feedback_gain, double translation_damping);
	    ~RotationController(){};

	    VectorXd getNewJointPositions( const DQ reference, const VectorXd thetas);
	    VectorXd getNewJointVelocities( const DQ reference, const VectorXd thetas);

	private: //methods	
		
	};
}