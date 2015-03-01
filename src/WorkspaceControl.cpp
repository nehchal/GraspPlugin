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
 * @file workspace.cpp
 * @author Can Erdogan, Saul Reynolds-Haertle
 * @date July 27, 2013
 * @brief This header file for workspace control of 7-dof arms.
 *
 * See the corresponding .hpp file for function descriptions
 */

#include "WorkspaceControl.hpp"

#include "util.hpp"

//#include "kore/display.hpp"
#include <amino/math.h>
#include <dynamics/BodyNode.h>
#include <math/MathTypes.h>

#define LEFT 0
#define RIGHT 1

//const int g_invType = LEFT;
 const int g_invType = RIGHT;

namespace Krang {

	/* ******************************************************************************************** */
	// WorkspaceControl::WorkspaceControl (dart::dynamics::Skeleton* robot, Side side, 
	WorkspaceControl::WorkspaceControl (dart::dynamics::BodyNode* endEffector,
	                                    double _K_posRef_p, double _nullspace_gain, double _damping_gain, 
	                                    double _ui_translation_gain, double _ui_orientation_gain,
	                                    double _compliance_translation_gain, double _compliance_orientation_gain) {

		// Determine the end-effector and the arm indices based on the input side
		//endEffector = robot->getBodyNode((side == LEFT) ? "lGripper" : "rGripper");
		this->endEffector = endEffector;
		Tref = endEffector->getWorldTransform();
	
		// Set the gains for control
		K_posRef_p = _K_posRef_p, nullspace_gain = _nullspace_gain, damping_gain = _damping_gain;

		// Set the gains for sensors
		ui_translation_gain = _ui_translation_gain, ui_orientation_gain = _ui_orientation_gain;
		compliance_translation_gain = _compliance_translation_gain;
		compliance_orientation_gain = _compliance_orientation_gain;
		
		// and don't print anything out
		debug_to_cout = false;
		return;
	}

	/* ******************************************************************************************** */
	void WorkspaceControl::resetReferenceTransform() {
		this->Tref = endEffector->getWorldTransform();
		return;
	}

	/* ******************************************************************************************** */
	void WorkspaceControl::setReferencePose(const Eigen::Vector6d& xRef){
		std::cout<<__LINE__<<": I am inside "<<__func__<<"()"<<std::endl;
		std::cout<<__LINE__<<": xRef = "<<xRef.transpose()<<std::endl;
		Tref = eulerToTransform(xRef);
		std::cout<<__LINE__<<": Tref = "<<transformToEuler(Tref).transpose()<<std::endl;
		//DISPLAY_MATRIX(Tref.matrix())
		return;
	}

	/* ******************************************************************************************** */
	void WorkspaceControl::integrateWSVelocityInput(const Eigen::Vector6d& xdot, const double dt) {

		// Represent the workspace velocity input as a 4x4 homogeneous matrix
		// Eigen::Matrix4d xdotM = eulerToTransform(xdot * dt);
		Eigen::Isometry3d xdotM = eulerToTransform(xdot * dt);
	
		// Compute the displacement in the end-effector frame with a similarity transform
		
		//Eigen::Matrix4d R = Tref;
		Eigen::Isometry3d R = Tref;
		//R.topRightCorner<3,1>().setZero();
		R.linear().setZero();
		//Eigen::Matrix4d Tdisp = R.inverse() * xdotM * R;
		Eigen::Isometry3d Tdisp = R.inverse() * xdotM * R;

		// Update the reference position for the end-effector with the given workspace velocity
		Tref = Tref * Tdisp;
	}

	/* ******************************************************************************************** */
	void WorkspaceControl::refWSVelocity(Eigen::Vector6d& xdot) {
		std::cout<<__LINE__<<": I am inside "<<__func__<<"()"<<std::endl;

		std::cout<<"Tref = " <<transformToEuler(Tref).transpose()<<std::endl;
		//DISPLAY_MATRIX(Tref.matrix())

		// Get the current end-effector transform and also, just its orientation 
		// Eigen::Matrix4d Tcur = endEffector->getWorldTransform();
		Eigen::Isometry3d Tcur = endEffector->getWorldTransform();

		std::cout<<"Tcur = "<<transformToEuler(Tcur).transpose()<<std::endl;
		DISPLAY_MATRIX(Tcur.matrix())

		//Eigen::Matrix4d Rcur = Tcur;
		Eigen::Isometry3d Rcur = Tcur;
		//Rcur.topRightCorner<3,1>().setZero();
		Rcur.translation().setZero();

		std::cout<<"Rcur = "<<std::endl;
		DISPLAY_MATRIX(Rcur.matrix())

		// Apply the similarity transform to the displacement between current transform and reference
		//Eigen::Matrix4d Tdisp = Tcur.inverse() * Tref;
		Eigen::Isometry3d Tdisp = Tcur.inverse() * Tref;
		//Eigen::Matrix4d xdotM = Rcur * Tdisp * Rcur.inverse();
		Eigen::Isometry3d xdotM = Rcur * Tdisp * Rcur.inverse();
		xdot = transformToEuler(xdotM) * K_posRef_p;
	}

	void WorkspaceControl::JSToWSVelocity(const Krang::Vector7d& qDot, 
														Eigen::Vector6d& xDot){
		dart::math::Jacobian JFull= endEffector->getWorldJacobian();	// this is 6 x 15 matrix
		dart::math::Jacobian J = JFull.topRightCorner<6, 7>();

		xDot = J * qDot;

		return;
	}

	/* ******************************************************************************************** */
	void WorkspaceControl::WSToJSVelocity(const Eigen::Vector6d& xdot,
	                                     	const Krang::Vector7d& qdot_nullspace, 
	                                     	  Krang::Vector7d& qdot) {
		// Get the Jacobian for the end-effector
		//Eigen::MatrixXd Jlin = endEffector->getJacobianLinear().topRightCorner<3,7>();
		//Eigen::MatrixXd Jang = endEffector->getJacobianAngular().topRightCorner<3,7>();
		//Eigen::MatrixXd J (6,7);
		//J << Jlin, Jang;
		//dart::math::Jacobian J = endEffector->getBodyJacobian();
		//std::cout<<__LINE__<<" rows in Bopdy Jacobian= "<<J.rows()<<" cols ="<<J.cols()<<std::endl;
		//std::cout<<"Body Jacobian ="<<std::endl;
		//DISPLAY_MATRIX(J)

		//dart::math::Jacobian JFull= endEffector->getWorldJacobian();	// this is 6 x 15 matrix
		dart::math::Jacobian JFull= endEffector->getBodyJacobian();	// this is 6 x 15 matrix
		dart::math::Jacobian J = JFull.topRightCorner<6, 7>();
		//dart::math::Jacobian J = JFull.topLeftCorner<6, 7>();
		
		// std::cout<<__LINE__<<" rows in World Jacobian= "<<J.rows()<<" cols ="<<J.cols()<<std::endl;
		//std::cout<<"World Jacobian ="<<std::endl;
		//DISPLAY_MATRIX(J)
		/*
		int n = endEffector->getNumDependentDofs();
		std::cout<<"Num of dependent dofs = "<<n<<std::endl;

		std::cout<<"indexes of dependent dofs are: ";
		for (int i=0; i<n; i++)
			std::cout<<endEffector->getDependentDof(i)<<" ";
		std::cout<<std::endl;

		std::cout << "DOF from effector: \n";
    	dart::dynamics::BodyNode* parent = endEffector;
    	while (parent != NULL) {
    	    std::cout << "Node name: " << parent->getName() << ", idx: " << parent->getSkeletonIndex() << std::endl;
    	    parent = parent->getParentBodyNode();
    	}
    	*/

		// Compute the inverse of the Jacobian with dampening
		Eigen::MatrixXd Jt = J.transpose();
		Eigen::MatrixXd Jinv;

		if(g_invType == LEFT){
			Eigen::MatrixXd temp = Jt * J;
			for(int i = 0; i < temp.rows(); i++) temp(i,i) += damping_gain;
			Jinv = (temp.inverse()) * Jt;
		}

		else {
			Eigen::MatrixXd temp = J * Jt;
			for(int i = 0; i < temp.rows(); i++) temp(i,i) += damping_gain;
			Jinv = Jt * (temp.inverse());
		}

		// Compute the joint velocities qdot using the input xdot and a qdot for the secondary goal 
		// projected into the nullspace
		///Eigen::MatrixXd JinvJ = Jinv*J;
		///Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7,7);
		
		Eigen::VectorXd qdot_jacobian_pure = Jinv * xdot;
		///Krang::Vector7d qdot_jacobian_null = (I - JinvJ) * qdot_nullspace * nullspace_gain;
		///qdot = qdot_jacobian_pure + qdot_jacobian_null;
		qdot = qdot_jacobian_pure;
		return;
	}

	/* ******************************************************************************************** */
	void WorkspaceControl::updateFromXdot (const Eigen::Vector6d& xdot, const Eigen::Vector6d& ft, 
	                                       const Krang::Vector7d& qdot_secondary, double dt, Krang::Vector7d& qdot) {

		// Move the workspace references around from that ui input
		integrateWSVelocityInput(xdot, dt);

		// Compute an xdot for complying with external forces if the f/t values are within thresholds
		Eigen::VectorXd xdot_comply(6);
		xdot_comply.topLeftCorner<3,1>() = -ft.topLeftCorner<3,1>() * compliance_translation_gain;
		xdot_comply.bottomLeftCorner<3,1>() = -ft.bottomLeftCorner<3,1>() * compliance_orientation_gain;

		// Get an xdot out of the P-controller that's trying to drive us to the refernece position
		Eigen::Vector6d xdot_posref;
		refWSVelocity(xdot_posref);

		// Combine the velocities from the workspace position goal, the ui, and the compliance
		Eigen::VectorXd xdot_apply = xdot_posref + xdot_comply;// + xdot;

		// Compute qdot with the dampened inverse Jacobian, using nullspace projection to achieve our 
		// secondary goal
		WSToJSVelocity(xdot_apply, qdot_secondary, qdot);

		// do debug printing to standard out if configured
		if (debug_to_cout) {
			DISPLAY_VECTOR(xdot_apply);
			DISPLAY_VECTOR(xdot_posref);
			DISPLAY_VECTOR(xdot_comply);
			DISPLAY_VECTOR(ft);
			DISPLAY_MATRIX(Tref.matrix());
			DISPLAY_VECTOR(xdot);
		}

		// do debug printing to standard out if configured
		// if (debug_to_curses) {
		// 	curses_display_vector(xdot_posref, "xdot from position ref");
		// 	curses_display_vector(xdot_comply, "xdot from compliance");
		// }
	}

	/* ******************************************************************************************** */
	void WorkspaceControl::updateFromUIVel(const Eigen::Vector6d& ui, const Eigen::Vector6d& ft, 
	                                       const Krang::Vector7d& qdot_secondary, double dt, Krang::Vector7d& qdot) {

		// turn our ui velocity input into a real velocity in workspace
		Eigen::VectorXd xdot_ui = this->uiInputVelToXdot(ui);

		// and then do the rest
		this->updateFromXdot(xdot_ui, ft, qdot_secondary, dt, qdot);
	}

	/* ******************************************************************************************** */
	Eigen::Vector6d WorkspaceControl::uiInputVelToXdot(const Eigen::Vector6d& ui_vel) {
		// Scale the ui input to get a workspace velocity 
		Eigen::VectorXd xdot_ui = ui_vel;
		xdot_ui.topLeftCorner<3,1>() *= ui_translation_gain;
		xdot_ui.bottomLeftCorner<3,1>() *= ui_orientation_gain;
		return xdot_ui;
	}


	/* ******************************************************************************************** */
	void WorkspaceControl::updateFromUIPos(const Eigen::Vector6d& xref, const Eigen::Vector6d& ft,
	                                       const Krang::Vector7d& qdot_secondary, Krang::Vector7d& qdot) {
		// update our workspace reference
		this->Tref = eulerToTransform(xref);

		// Compute an xdot for complying with external forces if the f/t values are within thresholds
		Eigen::VectorXd xdot_comply(6);
		xdot_comply.topLeftCorner<3,1>() = -ft.topLeftCorner<3,1>() * compliance_translation_gain;
		xdot_comply.bottomLeftCorner<3,1>() = -ft.bottomLeftCorner<3,1>() * compliance_orientation_gain;

		// Get an xdot out of the P-controller that's trying to drive us to the refernece position
		Eigen::Vector6d xdot_posref;
		refWSVelocity(xdot_posref);
		
		// Combine the velocities from the workspace position goal, the ui, and the compliance
		Eigen::VectorXd xdot_apply = xdot_posref + xdot_comply;// + xdot;

		// Compute qdot with the dampened inverse Jacobian, using nullspace projection to achieve our 
		// secondary goal
		WSToJSVelocity(xdot_apply, qdot_secondary, qdot);
	}

};	// end of namespace
