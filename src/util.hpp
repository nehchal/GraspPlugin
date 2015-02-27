/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice, this list of
 *       conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice, this list of
 *       conditions and the following disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived from this software without
 *       specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH RESEARCH
 * CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file util.hpp
 * @author Can Erdogan
 * Edited by Nehchal Jindal (Feb 2015)
 * @date July 25, 2013
 * @brief Generally useful things for people using kore
 *
 * This depends of version of DART previous to 2.6 . It has been edited to 
 conform to dart 3.0
 */



#pragma once

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
// #include <imud.h>
// #include <filter.h>
#include <ach.h>

#include <iomanip>

//#include <dynamics/SkeletonDynamics.h>
//#include <kinematics/BodyNode.h>
//#include <kinematics/Dof.h>
//#include <math/UtilsRotation.h>
#include <math/Geometry.h>
#include <simulation/World.h>

#include <Eigen/Dense>

namespace Krang {

/* ******************************************************************************************** */
// Useful math operations

/* Converts a 4x4 homogeneous transform to a 6D euler with the given order for RPY.
 * T : [IN] 4x4 homogeneous transfrom matrix 
 * returns 6-vector representing pose */
Eigen::VectorXd transformToEuler(const Eigen::Isometry3d &T);

/* Converts a 6D euler to a 4x4 homogeneous transform with the given order for RPY.
 * V : [IN] 6-vector representing pose
 * returns 4x4 homogeneous transfrom matrix  */
Eigen::Isometry3d eulerToTransform(const Eigen::VectorXd &V);

/* ******************************************************************************************** */
/// The indicator for the left or right side	
enum Side {
	LEFT = 0,
	RIGHT = 1
};

/* ******************************************************************************************** */
// IDs for the dart kinematic structure

extern std::vector <int> base_ids;					///< Ids for the lower body (x,y,th,imu,waist)
extern std::vector <int> kinect_ids;				///< Ids for the motors that control kinect
extern std::vector <int> left_arm_ids;			///< Ids for left arm 
extern std::vector <int> right_arm_ids;			///< Ids for right arm
extern std::vector <int> imuWaist_ids;			///< Ids for waist/imu
extern std::vector <int> dart_root_dof_ids; ///< Ids for the root position/orientation dofs in DART

/* ******************************************************************************************** */
// TODO: these should be in the Eigen namespace, not the Krang namespace.
typedef Eigen::Matrix<double, 6, 1> Vector6d;		///< A typedef to contain f/t values
typedef Eigen::Matrix<double, 7, 1> Vector7d;		///< A typedef to contain joint values
typedef Eigen::Matrix<double, 6, 6> Matrix6d;		///< A typedef to contain wrenches

/* ******************************************************************************************** */
// Useful macros

#define VELOCITY SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY
#define POSITION SOMATIC__MOTOR_PARAM__MOTOR_POSITION
#define pv(a) std::cout << #a << ": " << fix((a).transpose()) << "\n" << std::endl
#define pc(a) std::cout << #a << ": " << (a) << "\n" << std::endl
#define pm(a) std::cout << #a << ":\n " << fix((a).matrix()) << "\n" << std::endl
#define pmr(a) std::cout << #a << ":\n " << fix((a)) << "\n" << std::endl
#define parm (cout << llwa.pos[0] << ", " << llwa.pos[1] << ", " << llwa.pos[2] << ", " << \
          llwa.pos[3] << ", " << llwa.pos[4] << ", " << llwa.pos[5] << ", " << llwa.pos[6] << endl);
#define darm (cout << "dq: "<<llwa.vel[0] << ", " <<llwa.vel[1] << ", " << llwa.vel[2] << ", " << \
          llwa.vel[3] << ", " << llwa.vel[4] << ", " << llwa.vel[5] << ", " << llwa.vel[6] << endl);
#define eig7(x) (Vector7d() << (x)[0], (x)[1], (x)[2], (x)[3], (x)[4], (x)[5], (x)[6]).finished()

/* ********************************************************************************************* */
// as above, but nicely formatted. Prints fixed-point, fixed-width, and fixed-precision for 
// aligned columns and no exponents.
#define DISPLAY_SCALAR(SCA)                                             \
	{std::cout << std::setw(20) << std::left << #SCA; \
			std::cout << SCA << std::endl;}

#define DISPLAY_VECTOR(VEC)                                             \
	{std::cout << std::setw(20) << std::left << #VEC; \
			for(int i = 0; i < VEC.size(); i++) std::cout << std::setprecision(3) << std::fixed << std::setw(8) << VEC[i]; \
			std::cout << std::endl;}

#define DISPLAY_MATRIX(MAT)                                             \
	{std::cout << std::setw(20) << std::left << #MAT << std::endl; \
			for(int r = 0; r < MAT.rows(); r++) { \
				std::cout << "    "; \
				for(int c = 0; c < MAT.cols(); c++) { \
					std::cout << std::fixed << std::setw(12) << MAT(r, c); } \
				std::cout << std::endl; \
			} \
			std::cout << std::endl; }

/* ********************************************************************************************* */
};	// end of namespace

