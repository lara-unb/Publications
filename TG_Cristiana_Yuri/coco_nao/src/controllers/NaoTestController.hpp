#pragma once

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../DQ_controller.h"
#include <Eigen/Dense>
#include "../NAOSaver.hpp"

using namespace Eigen;



namespace DQ_robotics
{
	class TranslationController : public DQ_controller
	{
	public:

	private:
		DQ_kinematics robot_;
	    int robot_dofs_;

		//double rotation_damping_;
		MatrixXd translation_damping_;

	    float kp_;

		VectorXd j_mean; //media das juntas


	    VectorXd joints_; //joints


	    VectorXd error_translation_;
		//VectorXd error_rotation_;

	    MatrixXd analytical_jacobian_;
		
		//MatrixXd rotation_jacobian_;
	    MatrixXd translation_jacobian_pseudoinverse_;
		//MatrixXd rotation_jacobian_pseudoinverse_;

		MatrixXd nullspace_projector_;
		MatrixXd pseudo_inv_null_damped;
		VectorXd nullspace;

		MatrixXd identity4_;
		MatrixXd identityDOFS_;

	    DQ current_pose_; 


		DQ reference_translation_;
		DQ reference_rotation_;

		float epsilon;

		float s_;
		float s_bar;
		MatrixXd s_jacobian;

		int GAIN_S;



	public: //methods
		MatrixXd translation_jacobian_;
	    VectorXd delta_joints_; //variação das juntas

	    TranslationController(DQ_kinematics robot, float translation_feedback_gain, double translation_damping);
	    ~TranslationController(){};

	    VectorXd getNewJointPositions( const DQ reference, const VectorXd thetas);
	    VectorXd getNewJointVelocities( const DQ reference, const VectorXd thetas);



	private: //methods	
		
	};
}