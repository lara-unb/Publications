/**
Schunk Robot DH Parameters

\author Murilo Marques Marinho
\since 09/2013
*/

#ifndef DQ_ROBOTICS_A2ARM_DH_H
#define DQ_ROBOTICS_A2ARM_DH_H

#include"../DQ_kinematics.h"
#include<Eigen/Dense>
#include<cmath>


#define UPPER_ARM_LENGTH 0.105
#define ELBOW_YAW_OFFSET 0.015
#define LOWER_ARM_LENGTH 0.05595
#define SHOULDER_OFF_Y 0.098
#define SHOULDER_OFF_Z 0.1


using namespace Eigen;

namespace DQ_robotics{

    DQ_kinematics NAOKinematics_LArm()
    {
        const double pi2 = M_PI_2;
        const double pi  = M_PI;
        /*Matrix<double,4,5> a2arm_dh(4,5);
	    a2arm_dh <<  0,        pi2,      		  	0,     				0,    0,//theta
	                 0,    	   UPPER_ARM_LENGTH,  	0,		 			0,    LOWER_ARM_LENGTH,  //d
	                 0,        0,				  	ELBOW_YAW_OFFSET, 	0,    0,     //a
	                -pi2,      pi2,       		  	pi2, 		 	   -pi2,  pi2; //alpha*/

	   Matrix<double,5,7> a2arm_dh(5,7);

	    //Left Arm NAO_Robot
	   	a2arm_dh << 0,	  			0,        			pi2,      		  	0,     				0,    0,						-pi2,//theta
	                SHOULDER_OFF_Z, SHOULDER_OFF_Y,		0,  				UPPER_ARM_LENGTH,	0,    LOWER_ARM_LENGTH,			0,  //d
	                0,				0,       		 	ELBOW_YAW_OFFSET,	0, 					0,    0,     					0,//a
	                -pi2,	  		pi2,     			pi2,     		 	-pi2, 		 	    pi2,  -pi2, 					0,//alpha
	                1,	  			0,		  			0,					0,					0,	  0,						1;	 
	                
	    DQ_kinematics a2arm(a2arm_dh,"standard");

        return a2arm;        
    };

    DQ_kinematics NAOKinematics_RArm()
    {
        const double pi2 = M_PI_2;
        const double pi  = M_PI;
        /*Matrix<double,4,5> a2arm_dh(4,5);
	    a2arm_dh <<  0,        pi2,      		  	0,     				0,    0,//theta
	                 0,    	   UPPER_ARM_LENGTH,  	0,		 			0,    LOWER_ARM_LENGTH,  //d
	                 0,        0,				  	ELBOW_YAW_OFFSET, 	0,    0,     //a
	                -pi2,      pi2,       		  	pi2, 		 	   -pi2,  pi2; //alpha*/

	   Matrix<double,5,7> a2arm_dh(5,7);

	    //Left Arm NAO_Robot
	   	a2arm_dh << 0,	  				0,        			pi2,      		  	0,     				0,    0,					-pi2,	//theta
	                SHOULDER_OFF_Z, 	-SHOULDER_OFF_Y,    0,  				UPPER_ARM_LENGTH,	0,    LOWER_ARM_LENGTH,  	0, 		//d
	                0,					0,	        		-ELBOW_YAW_OFFSET,	0, 					0,    0,     				0,		//a
	                -pi2,	  			pi2,      			pi2,       		  	-pi2, 		 	    pi2,  -pi2, 				0,		//alpha
	                1,	  				0,		  			0,					0,					0,	  0,					1;	 
	                
	    DQ_kinematics a2arm(a2arm_dh,"standard");

        return a2arm;        
    };

}

#endif


/* DQ_kinematics NAOKinematics_LArm()
    {
        const double pi2 = M_PI_2;
        const double pi  = M_PI;
  

	   Matrix<double,5,6> a2arm_dh(5,6);

	    //Left Arm NAO_Robot
	   	a2arm_dh << 0,	  			0,        				pi2,      		  	0,     				0,    0,//theta
	                SHOULDER_OFF_Z, SHOULDER_OFF_Y,    	  	0,  				UPPER_ARM_LENGTH,	0,    LOWER_ARM_LENGTH,  //d
	                0,				0,        				ELBOW_YAW_OFFSET,	0, 					0,    0,     //a
	                -pi2,	  		pi2,	      			pi2,       		  	-pi2, 		 	    pi2,  0, //alpha
	                1,	  			0,		  				0,					0,					0,	  0;	 
	                
	    DQ_kinematics a2arm(a2arm_dh,"standard");

        return a2arm;        
    };

    DQ_kinematics NAOKinematics_RArm()
    {
        const double pi2 = M_PI_2;
        const double pi  = M_PI;


	   Matrix<double,5,6> a2arm_dh(5,6);

	    //Left Arm NAO_Robot
	   	a2arm_dh << 0,	  				0,        			pi2,      		  	0,     				0,    0,//theta
	                SHOULDER_OFF_Z, 	-SHOULDER_OFF_Y,    0,  				UPPER_ARM_LENGTH,	0,    LOWER_ARM_LENGTH,  //d
	                0,					0,	        		-ELBOW_YAW_OFFSET,	0, 					0,    0,     //a
	                -pi2,	  			pi2,      			pi2,       		  	-pi2, 		 	    pi2,  0, //alpha
	                1,	  				0,		  			0,					0,					0,	  0;	 
	                
	    DQ_kinematics a2arm(a2arm_dh,"standard");

        return a2arm;        
    };*/