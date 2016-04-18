/*
 * Copyright (C) 2016 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Nadia Figueroa
 * email:   nadia.figueroafernandez@epfl.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef BIMANUAL_DS_EXECUTION_H
#define BIMANUAL_DS_EXECUTION_H

#include <assert.h>
#include "MathLib/MathLib.h"
#include "bimanual_ds.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "CDDynamics.h"

using namespace MathLib;

class bimanual_ds_execution
{
public:
    bimanual_ds_execution();
    ~bimanual_ds_execution();

    void init(double dt, double Gamma, double DGamma, double Gain_A, double Gain_K_l, double Gain_K_r);
    void update();

    // EE functions
    void setCurrentEEStates(const tf::Pose& left_ee, const tf::Pose& right_ee);
    void getNextEEStates(tf::Pose& left_ee, tf::Pose& right_ee);

    // Real Object functions
    void setCurrentObjectPose(const tf::Pose& object_pose);
    void setCurrentObjectVelocity(const Eigen::Vector3d& object_velocity);
    void setCurrentObjectState(const tf::Pose& object_pose, const Eigen::Vector3d& object_velocity);

    // Virtual Object functions    
    void setInterceptPositions(const tf::Pose& object_intercept, const tf::Pose& left_intercept, const tf::Pose& right_intercept);
    void initializeVirtualObject();
    void getVirtualObjectPose(tf::Pose& vo_pose);

    // Type Conversions
    void tfPosToVector(const tf::Vector3& pos, Vector& p);
    void tfQuatToVector(const tf::Quaternion& quat, Vector& q);
    void tfQuatToQuat(const tf::Quaternion& quat, tf::Quaternion& q);
    Eigen::Vector3f d2qw(Eigen::Vector4f  q,  Eigen::Vector4f  dq);

private:

    bimanual_ds                 *vo_DS;

    // (Robot) Left EE State Variables
    Vector 						RPos_End_left;            // Position of left ee in Right robot's RF
    tf::Quaternion			    ROri_End_left;            // Orientation of left ee in right robot's RF
    Vector 						DRPos_End_left;           // Velocity of left ee
    Vector 						DDRPos_End_left;          // Acceleration of left ee

    // (Robot) Left EE Desired Command Variables
    Vector 						PosDesired_End_left;      // Desired position of left ee
    Vector						DPosDesired_End_left;     // Desired velocity of left ee
    Vector						DDPosDesired_End_left;    // Desired acceleration of left ee
    Vector						RPos_End_left_Open_Loop;  // Open Loop position command for left ee

    // (Robot) Right EE State Variables
    Vector 						RPos_End_right;           // Position of right ee in Right robot's RF
    tf::Quaternion				ROri_End_right;           // Orientation of right ee in right robot's RF
    Vector 						DRPos_End_right;          // Velocity of right ee
    Vector 						DDRPos_End_right;         // Acceleration of right ee

    // (Robot) Right EE Desired Command Variables
    Vector 						PosDesired_End_right;     // Desired position of right ee
    Vector 						DPosDesired_End_right;    // Desired velocity of right ee
    Vector 						DDPosDesired_End_right;   // Desired acceleration of right ee
    Vector						RPos_End_right_Open_Loop; // Open Loop position command for right ee

    // Virtual Object State Variables
    Vector						RPos_Intercept_left;      // Intercept position of left ee in right robot's RF
    tf::Quaternion			    ROri_Intercept_left;      // Intercept orientation of left ee in right robot's RF
    Vector						RPos_Intercept_right;     // Intercept position of right ee in right robot's RF
    tf::Quaternion				ROri_Intercept_right;     // Intercept orientation of right ee in right robot's RF

    Vector 						Position_VO;              // Position of Virtual Object
    Vector 						RPos_Intercept;           // Position of VO at intercept
    Vector						ROri_Intercept;           // Orientation of VO at intercept

    // Real Object State Variables
    Vector 						RPos_object;              // Position of Real Object in Right robot's RF
    Vector 						DRPos_object;             // Velocity of Real Object
    Vector 						DDRPos_object;            // Acceleration of Real Object
    Vector						ROri_object;              // Orientation of Real Object in Right robot's RF

    // Orientation Dymamics
    CDDynamics                  *angular_cddynamics;
    Vector                      desired_ang_vel;
    Vector                      filter_ang_vel;


    double                      dt;
    double                      slerp_t;

};


#endif // BIMANUAL_DS_EXECUTION_H
