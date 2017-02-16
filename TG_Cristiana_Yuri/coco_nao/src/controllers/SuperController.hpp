#pragma once

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../DQ_controller.h"
#include <Eigen/Dense>

using namespace Eigen;



namespace DQ_robotics
{
	class SuperController : public DQ_controller
	{
	public:

	private:
		DQ_kinematics robot_R_;
	    int robot_dofs_;
	    DQ_kinematics robot_L_;
	    int robot_dofs_L_;

		//double rotation_damping_;
		MatrixXd translation_damping_;

	    MatrixXd kp_;

	    VectorXd j_meanR;
		VectorXd j_meanL; //media das juntas


	    VectorXd joints_R_; //joints
	    VectorXd delta_joints_R_; //variação das juntas

	    VectorXd joints_L_; //joints
	    VectorXd delta_joints_L_; //variação das juntas

	    VectorXd joints_Aug_;
	   

	    VectorXd error_rotation_R_;
	    VectorXd error_rotation_L_;
	    VectorXd error_absolute_;
	    VectorXd error_distance_;
	    MatrixXd error_;
	    static MatrixXd integral_error_;

	    MatrixXd analytical_jacobian_R_;
	    MatrixXd analytical_jacobian_L_;
	    MatrixXd rotation_jacobian_R_;
	    MatrixXd rotation_jacobian_L_;

	    MatrixXd relative_jacobian_;
	    MatrixXd relative_jacobian_translation;
	    MatrixXd relative_jacobian_2;
	    MatrixXd ext_jacobian_x2;
	    MatrixXd absolute_jacobian;
	    MatrixXd absolute_jacobian_translation;
	    MatrixXd relative_jacobian_distance;

	    
	    MatrixXd main_jacobian_pseudoinverse_;

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

		float distance;

	public: //methods
		VectorXd delta_joints_Aug_;
		MatrixXd main_jacobian_;
	    SuperController(DQ_kinematics robot_armR, DQ_kinematics robot_armL, MatrixXd translation_feedback_gain, double translation_damping);
	    ~SuperController(){};

	    VectorXd getNewJointPositions( const DQ reference, const VectorXd thetas);
	    VectorXd getNewJointVelocities( const DQ reference, const VectorXd thetas);
		void setDistance(const float &distance);

	private: //methods	
		
	};
}