#pragma once

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../DQ_controller.h"
#include <Eigen/Dense>

using namespace Eigen;



namespace DQ_robotics
{
	class OneDOFController : public DQ_controller
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
	    

	    VectorXd error_distance_;
		//VectorXd error_rotation_;

	    MatrixXd analytical_jacobian_R_;
	    MatrixXd analytical_jacobian_L_;
	    MatrixXd relative_jacobian_;
	    MatrixXd relative_jacobian_translation;
	    
		//MatrixXd translation_jacobian_;
		//MatrixXd rotation_jacobian_;
	    MatrixXd distance_jacobian_pseudoinverse_;
		//MatrixXd rotation_jacobian_pseudoinverse_;

		MatrixXd nullspace_projector_;
		MatrixXd pseudo_inv_null_damped;
		VectorXd nullspace;

		MatrixXd identity4_;
		MatrixXd identityDOFS_, identityAug_;

	    DQ current_pose_R_;
	    DQ current_pose_L_; 
	    DQ relative_pose_;
	    DQ relative_pose_2_;
	    DQ absolute_pose_;


		DQ reference_translation_;
		//DQ reference_rotation_;
		float distance;

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
		VectorXd delta_joints_Aug_;
		MatrixXd relative_jacobian_distance;
	    OneDOFController(DQ_kinematics robot_armR, DQ_kinematics robot_armL, float translation_feedback_gain, double translation_damping);
	    ~OneDOFController(){};

	    //VectorXd getNewJointPositions( const float obj_pose, const VectorXd thetas);
	    //VectorXd getNewJointVelocities( const float obj_pose, const VectorXd thetas);

	    VectorXd getNewJointPositions( const DQ obj_pose, const VectorXd thetas);
	    VectorXd getNewJointVelocities( const DQ obj_pose, const VectorXd thetas);

	private: //methods	
		
	};
}