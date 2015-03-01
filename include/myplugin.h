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
 
#ifndef MYPLUGIN_H
#define MYPLUGIN_H

#include "ui_myplugin.h"

#include <grip/qtWidgets/GripTab.h>
#include <grip/osgGolems/ViewerWidget.h>
#include <grip/qtWidgets/TreeViewReturn.h>
#include <math.h>

#include "../src/WorkspaceControl.hpp"

class MyPlugin : public GripTab
{
    Q_OBJECT
    Q_INTERFACES(GripTab)

public:
     MyPlugin(QWidget *parent = 0);
    ~MyPlugin();

    void GRIPEventSceneLoaded();
    void GRIPEventSimulationBeforeTimestep();
    void GRIPEventSimulationAfterTimestep();
    void GRIPEventSimulationStart();
    void GRIPEventSimulationStop();
    void GRIPEventPlaybackBeforeFrame();
    void GRIPEventPlaybackAfterFrame();
    void GRIPEventPlaybackStart();
    void GRIPEventPlaybackStop();
    void GRIPEventTreeViewSelectionChanged();
    void Refresh();

private:
    Ui::MyPluginTab *ui;
    Krang::WorkspaceControl* wsControl;
    
    private slots:

    void printRobotInfo();
    void getLGripperPos(Eigen::Vector6d &x, Krang::Vector7d &q);
    void moveArm(const Eigen::VectorXd qDot, float t) ;
    void graspInit();
    void moveToWaypoint(const Eigen::Vector6d &xRel);
    void grasp(const Eigen::VectorXd &x);

    // unit test functions
    void test_moveArm();
    void test_getLGripperPos();
    void test_moveToWaypoint();
    void test_WSToJSVelocity();
    
    void addValue();
    void reduceValue();
    void continuousValue();
    void seeRobotInfo();
    void moveLeftArm();
    void setKrangStartConfig();
    const Eigen::VectorXd& refJSVelocity(const Eigen::VectorXd& xdot,
                                         const Eigen::VectorXd& qdot_nullspace,
                                         Eigen::VectorXd& qdot);

};

#endif // MYPLUGIN_H
