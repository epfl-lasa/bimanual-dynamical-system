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


#include "bimanual_ds_execution.h"

bimanual_ds_execution::bimanual_ds_execution()
{
    // Initialize all parameters/variables
    gTimeToReach=0.0;


    // (Robot) Left EE State Variables
    RPos_End_left.Resize(3);            // Position of left ee in Right robot's RF
    DRPos_End_left.Resize(3);           // Velocity of left ee
    DDRPos_End_left.Resize(3);          // Acceleration of left ee

    // (Robot) Left EE Desired Command Variables
    PosDesired_End_left.Resize(3);      // Desired position of left ee
    DPosDesired_End_left.Resize(3);     // Desired velocity of left ee
    DDPosDesired_End_left.Resize(3);    // Desired acceleration of left ee
    RPos_End_left_Open_Loop.Resize(3);  // Open Loop position command for left ee

    // (Robot) Right EE State Variables
    RPos_End_right.Resize(3);           // Position of right ee in Right robot's RF
    DRPos_End_right.Resize(3);          // Velocity of right ee
    DDRPos_End_right.Resize(3);         // Acceleration of right ee

    // (Robot) Right EE Desired Command Variables
    PosDesired_End_right.Resize(3);     // Desired position of right ee
    DPosDesired_End_right.Resize(3);    // Desired velocity of right ee
    DDPosDesired_End_right.Resize(3);   // Desired acceleration of right ee
    RPos_End_right_Open_Loop.Resize(3); // Open Loop position command for right ee

    // Virtual Object State Variables
    RPos_Intercept_left.Resize(3);      // Intercept position of left ee in right robot's RF
    RPos_Intercept_right.Resize(3);     // Intercept position of right ee in right robot's RF
    Position_VO.Resize(3);              // Position of Virtual Object
    ROri_Intercept.Resize(4);           // Orientation of VO at intercept
    RPos_Intercept.Resize(3);           // Position of VO at intercept
    RPos_Intercept.Zero();

    // Real Object State Variables
    RPos_object.Resize(3);              // Position of Real Object in Right robot's RF
    DRPos_object.Resize(3);             // Velocity of Real Object
    DDRPos_object.Resize(3);            // Acceleration of Real Object
    ROri_object.Resize(4);              // Orientation of Real Object in Right robot's RF

    RPos_object.Zero();
    DRPos_object.Zero();
    DDRPos_object.Zero();
    DDRPos_object(2)=-0.5;

    // Some default values
    dt = 0.002;
    reachingThr=0.005;

}

bimanual_ds_execution::~bimanual_ds_execution()
{
}


void bimanual_ds_execution::init(double dt, double Gamma, double DGamma, double Gain_A, double Gain_K_l, double Gain_K_r){

    vo_DS = new bimanual_ds();
    // Initialize ds motion parameters
    vo_DS->initialize(dt, Gamma, DGamma, Gain_A, Gain_K_l, Gain_K_r);    

    // Initialize left ee states to 0
    DRPos_End_left.Zero();
    DDRPos_End_left.Zero();

    // Initialize left ee states to 0
    DRPos_End_right.Zero();
    DDRPos_End_right.Zero();

}

void bimanual_ds_execution::setCurrentEEStates(const tf::Pose& left_ee, const tf::Pose& right_ee){

    // Convert Pose Types
    tfPosToVector (left_ee.getOrigin(), RPos_End_left);
    tfPosToVector (right_ee.getOrigin(), RPos_End_right);

    // Set Current Object States
    vo_DS->Set_Left_robot_state(RPos_End_left, DRPos_End_left, DDRPos_End_left);
    vo_DS->Set_Right_robot_state(RPos_End_right, DRPos_End_right, DDRPos_End_right);

}

void bimanual_ds_execution::setCurrentObjectPose(const tf::Pose& object_pose)
{
        // Convert Pose Types
        tfPosToVector (object_pose.getOrigin(),   RPos_object);
        tfQuatToVector(object_pose.getRotation(), ROri_object);

}

void bimanual_ds_execution::setCurrentObjectVelocity(const Eigen::Vector3d& object_velocity)
{

    double VosO_H[3];
    VosO_H[0]= object_velocity[0];
    VosO_H[1]= object_velocity[0];
    VosO_H[2]= object_velocity[0];
    DRPos_object.Set(VosO_H,3);

}

void bimanual_ds_execution::setCurrentObjectState(const tf::Pose& object_pose, const Eigen::Vector3d& object_velocity){

    setCurrentObjectPose(object_pose);
    setCurrentObjectVelocity(object_velocity);

}

void bimanual_ds_execution::setInterceptPositions(const tf::Pose& object_intercept, const tf::Pose& left_intercept, const tf::Pose& right_intercept){

    tfPosToVector (object_intercept.getOrigin(), RPos_Intercept);
    tfPosToVector (left_intercept.getOrigin()  , RPos_Intercept_left);
    tfPosToVector (right_intercept.getOrigin() , RPos_Intercept_right);

}

void bimanual_ds_execution::initializeVirtualObject(){

    ROri_Intercept = ROri_object;
    vo_DS->Set_object_Orien(ROri_object,ROri_Intercept);
    vo_DS->Set_object_state(RPos_object, DRPos_object, DDRPos_object, RPos_Intercept, RPos_Intercept_left, RPos_Intercept_right);
    vo_DS->initialize_Virrtual_object();
    ROS_INFO_STREAM("The virtual-object dynamical system is initialized!!");

}

void bimanual_ds_execution::getVirtualObjectPose(tf::Pose& vo_pose){

    Vector p = vo_DS->Get_virtual_object_pos();
    Vector q = vo_DS->Get_virtual_object_orie();
    vo_pose.setOrigin(tf::Vector3(p[0],p[1],p[2]));
    vo_pose.setRotation(tf::Quaternion(q[1],q[2],q[3],q[0]));

}

void bimanual_ds_execution::update(){
    
    // Update Object State and Intercept Positions
    vo_DS->Set_object_Orien(ROri_object,ROri_Intercept);
    vo_DS->Set_object_state(RPos_object, DRPos_object, DDRPos_object, RPos_Intercept, RPos_Intercept_left, RPos_Intercept_right);

    // Update Dynamical System Controller
    vo_DS->Update();

    // Get desired end-effector poses
    vo_DS->Get_Left_robot_state(PosDesired_End_left,DPosDesired_End_left,DDPosDesired_End_left);
    vo_DS->Get_Right_robot_state(PosDesired_End_right,DPosDesired_End_right,DDPosDesired_End_right);    

    // Set next states x_dot, x_ddot
    DRPos_End_left=DPosDesired_End_left;
    DDRPos_End_left=DDPosDesired_End_left;
    DRPos_End_right=DPosDesired_End_right;
    DDRPos_End_right=DDPosDesired_End_right;

}

void bimanual_ds_execution::getNextEEStates(tf::Pose &left_ee, tf::Pose &right_ee){

    // Convert Pose Types
    left_ee.setOrigin(tf::Vector3(PosDesired_End_left[0],PosDesired_End_left[1],PosDesired_End_left[2]));
    right_ee.setOrigin(tf::Vector3(PosDesired_End_right[0],PosDesired_End_right[1],PosDesired_End_right[2]));

}

void bimanual_ds_execution::tfPosToVector(const tf::Vector3& pos, Vector& p){

    assert(p.Size()==3);
    double p_[3];
    p_[0] =  pos.getX();
    p_[1] =  pos.getY();
    p_[2] =  pos.getZ();
    p.Set(p_,3);
}

void bimanual_ds_execution::tfQuatToVector(const tf::Quaternion& quat, Vector& q){

    assert(q.Size()==4);
    double q_[4];
    q_[0] =  quat.getW();
    q_[1] =  quat.getX();
    q_[2] =  quat.getY();
    q_[3] =  quat.getZ();
    q.Set(q_,4);
}
