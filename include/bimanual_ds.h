/*
 * Copyright (C) 2015 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Sina Mirrazavi
 * email:   sina.mirrazavi@epfl.ch
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

#include "MathLib/MathLib.h"
#include <stdio.h>
#include <stdlib.h>
#include "MathLib/IKGroupSolver.h"

enum ENUM_State{Com_Stop,Com_Break, Com_Safe};

enum ENUM_State_Orie{Not_Follow,Per_Follow};

using namespace MathLib;

class bimanual_ds
{
public:

	void 				initialize(double dt, double Gamma, double DGamma, double Gain_A, double  Gain_K_l, double Gain_K_r);
	void 				initialize_Virrtual_object();
	void				Set_object_state(Vector P_O,Vector DP_O, Vector DDP_O,Vector P_O_C,Vector P_O_C_L,Vector P_O_C_R);
	void 				Set_object_Orien(Vector O_O,Vector O_O_C);
	void				Set_Left_robot_state(Vector P_R_L,Vector DP_R_L, Vector DDP_R_L);
	void				Set_Right_robot_state(Vector P_R_R,Vector DP_R_R, Vector DDP_R_R);
	void				Get_Right_robot_state(Vector& _P_R_R,Vector & _DP_R_R, Vector & _DDP_R_R);
	void				Get_Left_robot_state(Vector & _P_R_L,Vector & _DP_R_L, Vector & _DDP_R_L);
	void				Update();
	void				Print_states();
	void				Debug(Matrix & _Data_Debug);
	ENUM_State			What_is_the_state();
	bool				What_is_the_Orie_state();
	Vector				Get_virtual_object_pos();
	Vector				Get_virtual_object_orie();
	double 				Getdt_();
	double 				GetGamma_();
private:

	void				Frame_transform_U_T_C();
	void 				initialize_Gains(Matrix & M, double Gain);
	void				Frame_transform_C_T_U();
	Matrix 				ConvertQuatsToMats(Vector Orie_Q);
	Vector 				ConvertQuatsToEula(Vector Orie_Q);

	double 				Gamma_;
	double 				DGamma_;
	double 				Gamma_O_;
	double 				dt_;
	double 				Break_;

	Matrix 				A_1_;
	Matrix 				K_r_;
	Matrix 				K_l_;
	Vector				U_l_;
	Vector				U_r_;


	Vector 				S_O_C_;
	Vector 				P_O_C_;
	Vector 				P_O_C_L_;
	Vector				P_C_O_C_L_;
	Vector 				P_U_O_C_L_;
	Vector 				P_U_VO_C_L_;
	Vector 				P_O_C_R_;
	Vector				P_C_O_C_R_;
	Vector 				P_U_O_C_R_;
	Vector 				P_U_VO_C_R_;
	Vector 				S_VO_C_L_;
	Vector 				S_VO_C_R_;

	Vector 				S_O_;
	Vector 				DS_O_;
	Vector 				P_O_;
	Vector 				DP_O_;
	Vector 				DDP_O_;
	Vector 				P_U_O_;
	Vector 				O_U_O_;
	Vector 				DO_O_;
	Vector 				O_U_O_old_;
	Vector				O_U_O_C_;
	Matrix				U_O_C_Matrix_;
	Matrix				U_O_VO_Matrix_;
	Matrix				C_O_U_Matrix_;
	Matrix				C_O_VO_Matrix_;


	Vector 				S_VO_;
	Vector 				DS_VO_;
	Vector 				P_VO_;
	Vector 				DP_VO_;
	Vector 				DDP_VO_;
	Vector				O_U_VO_;
	Vector				DO_VO_;
	Vector				O_O_C_Euler;

	Vector 				S_R_L_;
	Vector 				DS_R_L_;
	Vector 				P_R_L_;
	Vector 				DP_R_L_;
	Vector 				DDP_R_L_;
	Vector 				P_R_U_L_;

	Vector 				S_R_R_;
	Vector 				DS_R_R_;
	Vector 				P_R_R_;
	Vector 				DP_R_R_;
	Vector 				DDP_R_R_;
	Vector 				P_R_U_R_;

	ENUM_State			Command;
	ENUM_State_Orie		State_Orie;
	bool				FLag_of_command;
	bool				first_time_orientation;

};


