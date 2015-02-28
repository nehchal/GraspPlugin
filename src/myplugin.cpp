/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Sungmoon Joo <sungmoon.joo@gmail.com>
 * Date: Feb 2014
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of the Humanoid Robotics Lab nor the names of
 *     its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */
 
#include "myplugin.h"
#include "WorkspaceControl.hpp"
#include "util.hpp"
#include <iostream>
#include <qplugin.h>
#include <QtGui>
#include <dart/dynamics/Skeleton.h>
#include <dart/dynamics/Joint.h>
#include <dart/dynamics/BodyNode.h>
#include <math.h>
#include <Eigen/Dense>

#define DEG2RAD(x) x*3.1415/180

 // initializers for the workspace control constants
const double K_WORKERR_P = 1.00;
const double NULLSPACE_GAIN = 0.1;
const double DAMPING_GAIN = 0.005;
const double SPACENAV_ORIENTATION_GAIN = 0.50; // maximum 50 cm per second from spacenav
const double SPACENAV_TRANSLATION_GAIN = 0.25; // maximum .25 radians per second from spacenav
const double COMPLIANCE_TRANSLATION_GAIN = 1.0 / 750.0;
const double COMPLIANCE_ORIENTATION_GAIN = .125 / 750.0;
    
MyPlugin::MyPlugin(QWidget *parent) : ui(new Ui::MyPluginTab){
    ui->setupUi(this);


    // Button connections
    /*
    connect( ui->pushButtonAdd, SIGNAL(released()),
             this, SLOT(addValue()) );
    connect( ui->pushButtonReduce, SIGNAL(released()),
             this, SLOT(reduceValue()) );
    connect( ui->pushButtonSetStartConfiguration, SIGNAL(released()),
             this, SLOT(setKrangStartConfig()) );
    //connect( ui->seeRobotInfoButton, SIGNAL(released()),
    //         this, SLOT(seeRobotInfo()) );
    */

    connect( ui->seeRobotInfoButton, SIGNAL(released()),
             this, SLOT(printRobotInfo()) );    
    connect( ui->pushButtonMoveLeftArm, SIGNAL(released()),
	     this, SLOT(moveLeftArm()) );
    connect( ui->graspButton, SIGNAL(released()),
         this, SLOT(graspInit()) );
    
}

MyPlugin::~MyPlugin(){}

void MyPlugin::GRIPEventSimulationBeforeTimestep()
{
}

void MyPlugin::GRIPEventSimulationAfterTimestep(){}
void MyPlugin::GRIPEventSimulationStart(){}
void MyPlugin::GRIPEventSimulationStop(){}

/**
 * @function GRIPEventSceneLoaded
 * @brief Run right after an scene has been loaded
 */
void MyPlugin::GRIPEventSceneLoaded() {
    std::cout<<__LINE__<<": I am inside "<<__func__<<"()"<<std::endl;

    // Get GolemHubo skeleton
    /*
    dart::dynamics::Skeleton* skel = _world->getSkeleton("tetrapak");

    if (skel) {
        // Get index of LSP (left shoulder pitch
        std::vector<int> index(1);
        index[0] = skel->getJoint("LSP")->getGenCoord(0)->getSkeletonIndex();

        // Initialize joint value for LSP
        Eigen::VectorXd jointValue(1);

        // Move joint around
        for (size_t i = 0; i < 200; ++i) {
            // Set joint value
            jointValue[0] = float(i) * 2 * M_PI / 200;
            skel->setConfig(index, jointValue);

            // Save world to timeline
            _timeline->push_back(GripTimeslice(*_world));
        }
    } else {
        std::cerr << "No skeleton named lwa4" << std::endl;
    }
    */
    dart::dynamics::Skeleton* robotSkeleton = _world->getSkeleton("Krang");
    wsControl = new Krang::WorkspaceControl(
                        robotSkeleton->getBodyNode("lGripper"), Krang::LEFT,
                        K_WORKERR_P, NULLSPACE_GAIN, DAMPING_GAIN, 
                        SPACENAV_TRANSLATION_GAIN, SPACENAV_ORIENTATION_GAIN,
                        COMPLIANCE_TRANSLATION_GAIN, COMPLIANCE_ORIENTATION_GAIN);
    return;
}

void MyPlugin::GRIPEventTreeViewSelectionChanged()
{
    dart::dynamics::Skeleton* robot = _world->getSkeleton("Krang");    
    if(!_activeNode) {
        std::cerr << "[MyPlugin] No item selected in TreeView" << std::endl;
        return;
    }

    std::cerr << "[MyPlugin] ActiveNodeType: " << _activeNode->dType << std::endl;
    if(Return_Type_Robot == _activeNode->dType) {
        dart::dynamics::Skeleton* robot = (dart::dynamics::Skeleton*)_activeNode->object;
        std::cerr << "[MyPlugin] Skeleton Selected: " << robot->getName() << std::endl;
    } else if(Return_Type_Node == _activeNode->dType) {
        dart::dynamics::BodyNode* node = (dart::dynamics::BodyNode*)_activeNode->object;
        std::cerr << "[MyPlugin] BodyNode Selected: " << node->getName() << std::endl;
    } else {
        std::cerr << "[MyPlugin] Unknown type selected in TreeView" << std::endl;
    }
}

void MyPlugin::GRIPEventPlaybackBeforeFrame() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed after every playback time step
 */
void MyPlugin::GRIPEventPlaybackAfterFrame() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the start of the playback
 */
void MyPlugin::GRIPEventPlaybackStart() {}

/**
 * \brief called from the main window whenever the simulation history slider is being played
 * This method is executed at the end of the playback
 */
void MyPlugin::GRIPEventPlaybackStop() {}

void MyPlugin::Refresh() {}

void MyPlugin::printRobotInfo() {
    std::cout<<__LINE__<<"I am inside "<<__func__<<"()"<<std::endl;

    dart::dynamics::Skeleton *robotSkeleton = _world->getSkeleton("Krang");

    Eigen::VectorXd cfg = robotSkeleton->getConfig();
    Eigen::VectorXd cfg1 = _world->getState();

    std::cout<<"Robot configuration:"<<cfg.transpose()<<std::endl;
    std::cout<<"world state\n"<<cfg1.transpose()<<std::endl;

    // print the names of body nodes
    std::cout << "Full skeleton names: \n";
    for (int i=0; i < robotSkeleton->getNumBodyNodes(); i++) {
        std::cout << "idx: " << robotSkeleton->getBodyNode(i)->getSkeletonIndex() << ", name: " << robotSkeleton->getBodyNode(i)->getName() << std::endl;
    }

    dart::dynamics::BodyNode* lGripperBodyNode = robotSkeleton->getBodyNode("lGripper");
    std::cout<<"Pose of Left Gripper in World Frame:"<<std::endl;
    Eigen::Isometry3d T_lGripper_world = lGripperBodyNode->getWorldTransform();
    std::cout<<T_lGripper_world.matrix()<<std::endl;

    // Get the Jacobian for the end-effector
    // Eigen::MatrixXd Jfull = endEffector->getBodyJacobian();

    return;
}

/**
 * \brief This functions returns current position of left gripper.
 * x : [OUT] 6-vector representing position of left gripper in world frame
       in order (x, y, z, euler angles) */
void MyPlugin::getLGripperPos(Eigen::Vector6d x, Krang::Vector7d q) {
    std::cout<<__LINE__<<"I am inside "<<__func__<<"()"<<std::endl;

    dart::dynamics::Skeleton* robotSkeleton = _world->getSkeleton("Krang");
    dart::dynamics::BodyNode* lGripperBodyNode = 
                        robotSkeleton->getBodyNode("lGripper");
    // Pose of Left Gripper in World Frame
    Eigen::Isometry3d T_lGripper_world = lGripperBodyNode->getWorldTransform();
    
    // Convert trasform to state vector
    x = Krang::transformToEuler(T_lGripper_world);
    q = robotSkeleton->getConfig( *(wsControl->arm_ids));
}

/**
 * \brief This functions is used to go to a given waypoint
 * qDot : [IN] 7-vector joint velocity 
 * t    : [IN] time of the movement */
void MyPlugin::moveArm(const Eigen::VectorXd qDot, float t) {
    dart::dynamics::Skeleton* robotSkeleton = _world->getSkeleton("Krang");
    Eigen::VectorXd q = robotSkeleton->getConfig(* wsControl->arm_ids);

    q = q + t * qDot;
    robotSkeleton->setConfig(* wsControl->arm_ids, q);

    return;
}

/**
 * \brief This functions is used to go to a given waypoint
 * xRef : [IN] 6-vector pose of waypoint (reference position for end-effector) 
          in the world frame */
void MyPlugin::moveToWaypoint(const Eigen::VectorXd &xRef) {
    
    Eigen::Vector6d x;   // current pose in world frame
    Eigen::VectorXd q;   // current pose in joint space
    
    getLGripperPos(x, q);

    Eigen::VectorXd xDot;
    Eigen::VectorXd qDot;

    // get the state of the robot (the position of end-effector)
    /* First six values in _world->getState() and  
    _world->getSkeleton("Krang")->getConfig() are same which is the pose of the
    Krang robot. [0-5] maps to (x, y, z, roll, pitch, yaw)
    See grip2/.../grip_interface.py 
    Also see util.cpp for index details in config */

    // cfg contains joint angles. Convert to task position using FK

    // Set the reference position inside kore to be x
    wsControl->setReferencePose(xRef);
    wsControl->refWSVelocity(xDot);     // get task space velocity
    
    // get null space bias for jacobian
    Krang::Vector7d nullspace_q_ref;
    nullspace_q_ref <<  0, -1.0, 0, -0.5, 0, -0.8, 0;

    Krang::Vector7d nullspace_q_mask;
    nullspace_q_mask << 0, 0, 0, 1, 0, 0, 0;

    Krang::Vector7d nullspace_qDot_ref;
    nullspace_qDot_ref = (nullspace_q_ref  - q).cwiseProduct(nullspace_q_mask);

    // get joint space velocity
    wsControl->WSToJSVelocity(xDot, nullspace_qDot_ref, qDot);
    return;
}

/**
 * \brief This function initializes the WorkspaceControl class in kore library
 */
void MyPlugin::graspInit() {

    /* TODO: model of robot is required, from where to get it? Grip should have
    one. How to access that? */
    //wsCtrl = new Krang::WorkspaceControl()
    Eigen::VectorXd x(3);
    x[0] = 0.0;
    x[1] = 0.0;
    x[2] = 0.0;
    std::cout<<"I am inside graspInit()"<<std::endl;
    moveToWaypoint(x);
    return;
}

void MyPlugin::addValue() {
    Eigen::VectorXd newValues(1);
    std::vector<int> indices(1);
    indices[0] = 2;
    dart::dynamics::Skeleton* robot = _world->getSkeleton("Krang");
    Eigen::VectorXd altValues = robot->getConfig(indices);
    newValues(0) = altValues(0) +  0.1;
    robot->setConfig(indices, newValues);
}

void MyPlugin::reduceValue() {
    dart::dynamics::Skeleton* robot = _world->getSkeleton("Krang");
    Eigen::VectorXd newValues(1);
    std::vector<int> indices(1);
    indices[0] = 13;
    Eigen::VectorXd altValues = robot->getConfig(indices);
    newValues(0) = altValues(0) -  0.1;
    robot->setConfig(indices, newValues);
}

void MyPlugin::setKrangStartConfig() {
    dart::dynamics::Skeleton* robot = _world->getSkeleton("Krang");
    Eigen::VectorXd cfg = robot->getConfig();
    cfg << 1.57665,  -1.58573, -0.905034,  0.165138,   0.23035,  0.134083,        0 ,        0,  0.736354,         0,         0,         0 ,        0, -0.824843,         0, -0.761487,        0,  -1.14214,         0,         0 ,        0     ,    0 , 0.697957  ,       0  , 1.01526   ,      0 ,  1.14214 ,        0     ,    0    ,     0;
    robot->setConfig(cfg);
}

/**
void MyPlugin::moveLeftArm() {
    dart::dynamics::Skeleton* robot = _world->getSkeleton("Krang");
    Eigen::VectorXd newValues(1);
    std::vector<int> indices(1);
    indices[0] = 13;
    Eigen::VectorXd altValues = robot->getConfig(indices);
    newValues(0) = altValues(0) -  0.1;
    robot->setConfig(indices, newValues);
}
**/


/***********************************************************************
OBSOLETE

All functions after this line were written by Osayame and are now 
obsolete.
 - [Nehchal Feb 27, 2015]
***********************************************************************/

void MyPlugin::moveLeftArm() {
    //Initialize vectors
    Eigen::Vector6d xdot(6);
    xdot[0] = 1.0; // ask for +x velocity
    Eigen::VectorXd qdot;
    Eigen::VectorXd _nullspace;
    std::vector<int> left_arm_indices(7);
    Eigen::VectorXd l_arm_cfg(qdot.size());



    //Get qdot - velocity in joint, given x velocity in workspace
    refJSVelocity(xdot, _nullspace, qdot);

    //Print out xdot and qdot
    std::cout << "xdot: " << xdot.transpose() << std::endl;
    std::cout << "qdot: " << qdot.transpose() << std::endl;

    //Robot confiiguration
    dart::dynamics::Skeleton* robot = _world->getSkeleton("Krang");
    Eigen::VectorXd cfg = robot->getConfig();
    std::cout << "cfg size: " << cfg.size() << std::endl;
    std::cout << "qdot size: " << qdot.size() << std::endl;
    std::cout << "old cfg: " << cfg.transpose() << std::endl;

    // Initialize parameters
    double inc_by = 0.05;
    int l_shoulder_idx = 10;

    //Set left arm indices
    for(int i = 0; i < left_arm_indices.size(); i++)
        left_arm_indices[i] = l_shoulder_idx + i;

    //Set left arm configuration
    for(int i = 0; i < left_arm_indices.size(); i++) 
        //cfg[left_arm_indices[i]] += (inc_by * qdot[i]);
        l_arm_cfg[i] = cfg[left_arm_indices[i]] + (inc_by * qdot[i]);

    //Print out left arm indices and configuration info
    std::cout << "left arm indices: ";
    for (int i=0; i<left_arm_indices.size(); i++) 
        std::cout <<  " " << left_arm_indices[i] << std::endl;
    std::cout << "new cfg: " << cfg.transpose() << std::endl;
    std::cout << "indices len: " << left_arm_indices.size() << std::endl;
    // std::cout<< "cfg slice len: " << cfg.segment(l_shoulder_idx, l_shoulder_idx+6).size() << std::endl;
    // std::cout<< "cfg slice len: " << cfg.segment(10, 20).size() << std::endl;

    //robot->setConfig(left_arm_indices, cfg.segment(l_shoulder_idx, l_shoulder_idx+6));

    //Set new configuration
    robot->setConfig(left_arm_indices, l_arm_cfg);
}


const Eigen::VectorXd& MyPlugin::refJSVelocity(const Eigen::VectorXd& xdot,
                                         const Eigen::VectorXd& qdot_nullspace, Eigen::VectorXd& qdot) {


    dart::dynamics::Skeleton* robot = _world->getSkeleton("Krang");
    dart::dynamics::BodyNode* endEffector = robot->getBodyNode("lGripper");

    // Get the Jacobian for the end-effector
    Eigen::MatrixXd Jfull = endEffector->getBodyJacobian();
    Eigen::MatrixXd J = Jfull.topRightCorner<6,7>();
    // Eigen::MatrixXd J = Jfull.topLeftCorner<6,7>(); //???


    // debugging
    std::cout << "DOF from effector: \n";
    dart::dynamics::BodyNode* parent = endEffector;
    while (parent != NULL) {
        std::cout << "Node name: " << parent->getName() << ", idx: " << parent->getSkeletonIndex() << std::endl;
        parent = parent->getParentBodyNode();
    }

    std::cout << "Full skeleton names: \n";
    for (int i=0; i < robot->getNumBodyNodes(); i++) {
        std::cout << "idx: " << robot->getBodyNode(i)->getSkeletonIndex() << ", name: " << robot->getBodyNode(i)->getName() << std::endl;
    }

    // Transposing the Jacobian
    Eigen::MatrixXd Jt = J.transpose();
    Eigen::MatrixXd JJt = J * Jt;
    // std::cout << "j dimensions: " << J.rows() << "x" << J.cols() << std::endl;
    // std::cout << "jt dimensions: " << Jt.rows() << "x" << Jt.cols() << std::endl;
    // std::cout << "jjt dimensions: " << JJt.rows() << "x" << JJt.cols() << std::endl;
    // std::cout << "jjt coefficients: " << JJt.size() << std::endl;

    // // inverting the jacobian
    Eigen::MatrixXd Jinv = Jt * JJt.inverse();
    // std::cout << "right pseudo-inverse: " << Jinv << std::endl;
    qdot = Jinv * xdot;
    std::cout << "final qdot from rpi: " << qdot.transpose() << std::endl;

    // Eigen version:
    Eigen::VectorXd qdot2 = J.colPivHouseholderQr().solve(xdot);
    std::cout << "final qdot from eigen QR: " << qdot2.transpose() << std::endl;
    std::cout << "err: " << (qdot - qdot2).transpose() << std:: endl;

    return qdot;
}



void MyPlugin::seeRobotInfo() {
    std::cout<<__LINE__<<"I am inside "<<__func__<<"()"<<std::endl;
    dart::dynamics::Skeleton* skel = _world->getSkeleton("Krang");
    dart::dynamics::Skeleton* table = _world->getSkeleton("glassDiningTable");
    dart::dynamics::BodyNode* node = skel->getBodyNode("L2");
    if (skel != NULL) {
    std::cout<<"Robot name: "<<skel->getName()<<std::endl;
        Eigen::VectorXd robotConfig = skel->getConfig();
        std::cout<<"Robot configuration: "<<robotConfig.matrix().transpose()<<std::endl;
    std::cout<<"Left arm position: "<<node->getWorldJacobian().matrix().transpose()<<std::endl;
        
        std::cout<<"printing left joint info: "<<skel->getJoint("LJ2")->getGenCoord(0)->get_q()<<std::endl;
        
        Eigen::VectorXd tableConfig = table->getConfig();
        std::cout<<"printing table configuration: "<<tableConfig.transpose()<<std::endl;
       
    Eigen::Isometry3d tableWorld = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d grasppositionTable = Eigen::Isometry3d::Identity();

        tableWorld = table->getBodyNode(0)->getWorldTransform();
    grasppositionTable.translation()<<0.21, 0, 0.32;

    Eigen::Isometry3d grasppositionWorld;
    grasppositionWorld = tableWorld * grasppositionTable;
        
             
    std::cout<<"printing table info: \n"<<grasppositionWorld.matrix()<<std::endl; 
        
    Eigen::Isometry3d gripWorld = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d gripperGrip = Eigen::Isometry3d::Identity();
     
        gripWorld = skel->getBodyNode("lGripper")->getWorldTransform();
    gripperGrip.translation()<<0, 0, 0.16;

    Eigen::Isometry3d gripperWorld;
    gripperWorld  = gripWorld * gripperGrip;

        std::cout<<"printing grip info :  \n"<<gripperWorld.matrix()<<std::endl;
        
        Eigen::Vector3d distance;
    distance = gripperWorld.translation() - grasppositionWorld.translation();

        
        std::cout<<"printing distance info :  \n"<<distance.norm()<<std::endl;     
 }
}






Q_EXPORT_PLUGIN2(MyPlugin, MyPlugin)
