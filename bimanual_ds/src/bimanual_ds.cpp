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


#include "bimanual_ds.h"


Matrix bimanual_ds::ConvertQuatsToMats(Vector Orie_Q) {
	Matrix Rotation_Matrix;
	Rotation_Matrix.Resize(3,3);
	double q[4];
	q[3]=Orie_Q(0);q[1]=Orie_Q(2);
	q[2]=Orie_Q(3);q[0]=Orie_Q(1);
	double xx = q[0]*q[0], xy = q[0]*q[1], xz = q[0]*q[2], xw = q[0]*q[3];
	double yy = q[1]*q[1], yz = q[1]*q[2], yw = q[1]*q[3];
	double zz = q[2]*q[2], zw = q[2]*q[3];
	Rotation_Matrix(0,0)=1 - 2 * yy - 2 * zz;	Rotation_Matrix(0,1)=2 * xy - 2 * zw;	Rotation_Matrix(0,2)=2 * xz + 2 * yw;
	Rotation_Matrix(1,0)=2 * xy + 2 * zw;	Rotation_Matrix(1,1)=1 - 2 * xx - 2 * zz;	Rotation_Matrix(1,2)=2 * yz - 2 * xw;
	Rotation_Matrix(2,0)=2 * xz - 2 * yw;	Rotation_Matrix(2,1)=2 * yz + 2 * xw;	Rotation_Matrix(2,2)=1 - 2 * xx - 2 * yy;

	return Rotation_Matrix;

}

Vector bimanual_ds::ConvertQuatsToEula(Vector Orie_Q) {
	Vector Euler;
	double q0=Orie_Q(0);	double q1=Orie_Q(1);	double q2=Orie_Q(2);	double q3=Orie_Q(3);
	Euler.Resize(3);
	Euler(0)= atan2(2 * (q0*q1 + q2*q3), 1 - 2 * (q1*q1 + q2*q2));
	Euler(1)= asin( 2 * (q0*q2 - q3*q1));
	Euler(2)=atan2(2 * (q0*q3 + q1*q2), 1 - 2 * (q2*q2 + q3*q3));
	return Euler;
}


void bimanual_ds::initialize_Gains(Matrix & M, double Gain)
{
	M.Zero();
	M(0,3)=1;
	M(1,4)=1;
	M(2,5)=1;
	M(0,3)=1;
	double Scaler_V=1;
	double Scaler_Z=1;
	M(3,0)=-1*Gain*Gain;
	M(4,1)=-1*Scaler_V*Scaler_V*Gain*Gain;
	M(5,2)=-1*Scaler_Z*Scaler_Z*Gain*Gain;

	M(3,3)=-2*Gain;
	M(4,4)=-2*Scaler_V*Gain;
	M(5,5)=-2*Scaler_Z*Gain;
}


void bimanual_ds::Print_states()
{
	S_VO_.Print("S_VO_");
	DS_VO_.Print("DS_VO_");

	S_O_.Print("S_O_");
	DS_O_.Print("DS_O_");

	U_l_.Print("U_l_");
	U_r_.Print("U_r_");

	cout<<Gamma_<<" Gamma_"<<endl;

}

void bimanual_ds::initialize(double dt, double Gamma, double DGamma, double Gain_A, double  Gain_K_l, double Gain_K_r)
{
	dt_=dt;
	Gamma_=Gamma;
	DGamma_=DGamma;

	Gamma_O_=0.0;

	Break_=1-0.0001;

	A_1_.Resize(6,6);
	K_r_.Resize(6,6);
	K_l_.Resize(6,6);
	U_l_.Resize(6);
	U_r_.Resize(6);

	S_O_.Resize(6);
	DS_O_.Resize(6);
	P_O_.Resize(3);
	DP_O_.Resize(3);
	DDP_O_.Resize(3);
	P_U_O_.Resize(3);

	S_VO_.Resize(6);
	DS_VO_.Resize(6);
	P_VO_.Resize(3);
	DP_VO_.Resize(3);
	DDP_VO_.Resize(3);

	O_U_O_.Resize(4);
	DO_O_.Resize(4);
	O_U_VO_.Resize(4);
	DO_VO_.Resize(4);
	O_U_O_old_.Resize(4);
	O_U_O_C_.Resize(4);
	U_O_C_Matrix_.Resize(3,3);
	U_O_VO_Matrix_.Resize(3,3);
	C_O_U_Matrix_.Resize(3,3);
	C_O_VO_Matrix_.Resize(3,3);

	S_R_L_.Resize(6);
	DS_R_L_.Resize(6);
	P_R_L_.Resize(3);
	DP_R_L_.Resize(3);
	DDP_R_L_.Resize(3);
	P_R_U_L_.Resize(3);

	S_R_R_.Resize(6);
	DS_R_R_.Resize(6);
	P_R_R_.Resize(3);
	DP_R_R_.Resize(3);
	DDP_R_R_.Resize(3);
	P_R_U_R_.Resize(3);

	S_O_C_.Resize(6);
	P_O_C_.Resize(3);

	P_O_C_L_.Resize(3);
	P_C_O_C_L_.Resize(3);
	P_O_C_R_.Resize(3);
	P_C_O_C_R_.Resize(3);
	P_U_O_C_L_.Resize(3);
	P_U_O_C_R_.Resize(3);
	P_U_VO_C_L_.Resize(3);
	P_U_VO_C_R_.Resize(3);
	S_VO_C_L_.Resize(6);
	S_VO_C_R_.Resize(6);

	initialize_Gains(A_1_,Gain_A);
	initialize_Gains(K_l_,Gain_K_l);
	initialize_Gains(K_r_,Gain_K_r);
	A_1_.Print("A_1_");


	Command=Com_Safe;
	State_Orie=Not_Follow;
	first_time_orientation=false;

/*	int argc=0;
    char * argv[1];
	ros::init(argc, argv, "Debug");
	ros::NodeHandle nh;
	pub_Debug = nh.advertise<geometry_msgs::Pose>("/Debug", 3);*/
}

void bimanual_ds::Set_object_Orien(Vector O_O,Vector O_O_C)
{
	if ((O_O.Size()!=4)&&(O_O_C.Size()!=4))
	{
		cout<<"The Orientation must be represented in quaternion  w<<x<<y<<z. "<<endl;
	}
	if (first_time_orientation==false)
	{
		O_U_O_=O_O;
		O_U_O_old_=O_O;
		first_time_orientation=true;
	}
	else
	{
		O_U_O_old_=O_U_O_;
		O_U_O_=O_O;
	}
	O_U_O_C_=O_O_C;
	DO_O_=O_U_O_-O_U_O_old_;
	/*	DO_O_.Mult(1/dt_,DO_O_);*/
}

void bimanual_ds::Set_object_state(Vector P_O,Vector DP_O, Vector DDP_O,Vector P_O_C,Vector P_O_C_L,Vector P_O_C_R)
{
	P_O_C_=P_O_C;
	P_U_O_C_L_=P_O_C_L;
	P_U_O_C_R_=P_O_C_R;
	P_U_O_=P_O;
	DP_O_=DP_O;
	DDP_O_=DDP_O;
}

void bimanual_ds::Set_Right_robot_state(Vector P_R_R,Vector DP_R_R, Vector DDP_R_R)
{
	P_R_U_R_=P_R_R;
	DP_R_R_=DP_R_R;
	DDP_R_R_=DDP_R_R;
}

void bimanual_ds::Set_Left_robot_state(Vector P_R_L,Vector DP_R_L, Vector DDP_R_L)
{
	P_R_U_L_=P_R_L;
	DP_R_L_=DP_R_L;
	DDP_R_L_=DDP_R_L;
}

void bimanual_ds::Get_Left_robot_state(Vector& _P_R_L,Vector& _DP_R_L, Vector& _DDP_R_L)
{
	_P_R_L=P_R_U_L_;
	_DP_R_L=DP_R_L_;
	_DDP_R_L=DDP_R_L_;

}

void bimanual_ds::Get_Right_robot_state(Vector& _P_R_R,Vector& _DP_R_R, Vector& _DDP_R_R)
{
	_P_R_R=P_R_U_R_;
	_DP_R_R=DP_R_R_;
	_DDP_R_R=DDP_R_R_;
}

void bimanual_ds::Frame_transform_U_T_C()
{

	P_R_R_=P_R_U_R_-P_O_C_;


	S_R_R_(0)=P_R_R_(0);	S_R_R_(1)=P_R_R_(1);	S_R_R_(2)=P_R_R_(2);
	S_R_R_(3)=DP_R_R_(0);	S_R_R_(4)=DP_R_R_(1);	S_R_R_(5)=DP_R_R_(2);

	DS_R_R_(0)=DP_R_R_(0);	DS_R_R_(1)=DP_R_R_(1);	DS_R_R_(2)=DP_R_R_(2);
	DS_R_R_(3)=DDP_R_R_(0);	DS_R_R_(4)=DDP_R_R_(1);	DS_R_R_(5)=DDP_R_R_(2);


	P_R_L_=P_R_U_L_-P_O_C_;


	S_R_L_(0)=P_R_L_(0);	S_R_L_(1)=P_R_L_(1);	S_R_L_(2)=P_R_L_(2);
	S_R_L_(3)=DP_R_L_(0);	S_R_L_(4)=DP_R_L_(1);	S_R_L_(5)=DP_R_L_(2);


	DS_R_L_(0)=DP_R_L_(0);	DS_R_L_(1)=DP_R_L_(1);	DS_R_L_(2)=DP_R_L_(2);
	DS_R_L_(3)=DDP_R_L_(0);	DS_R_L_(4)=DDP_R_L_(1);	DS_R_L_(5)=DDP_R_L_(2);


	S_VO_(0)=P_VO_(0);		S_VO_(1)=P_VO_(1);	    S_VO_(2)=P_VO_(2);
	S_VO_(3)=DP_VO_(0);		S_VO_(4)=DP_VO_(1);		S_VO_(5)=DP_VO_(2);

	DS_VO_(0)=DP_VO_(0);	DS_VO_(1)=DP_VO_(1);	DS_VO_(2)=DP_VO_(2);
	DS_VO_(3)=DDP_VO_(0);	DS_VO_(4)=DDP_VO_(1);	DS_VO_(5)=DDP_VO_(2);


	P_O_=P_U_O_-P_O_C_;

	S_O_(0)=P_O_(0);	S_O_(1)=P_O_(1);	S_O_(2)=P_O_(2);
	S_O_(3)=DP_O_(0);	S_O_(4)=DP_O_(1);	S_O_(5)=DP_O_(2);

	DS_O_(0)=DP_O_(0);	DS_O_(1)=DP_O_(1);	DS_O_(2)=DP_O_(2);
	DS_O_(3)=DDP_O_(0);	DS_O_(4)=DDP_O_(1);	DS_O_(5)=DDP_O_(2);


	P_C_O_C_L_=P_U_O_C_L_-P_O_C_;
	P_C_O_C_R_=P_U_O_C_R_-P_O_C_;

	P_O_C_L_=C_O_VO_Matrix_*P_C_O_C_L_;
	P_O_C_R_=C_O_VO_Matrix_*P_C_O_C_R_;


	/*	P_O_C_L_.Print("P_O_C_L_");
	P_O_C_R_.Print("P_O_C_R_");*/

	//	C_O_VO_Matrix_.Print("C_O_VO_Matrix_");

	if ((P_O_C_L_(1)>0)&&(P_O_C_R_(1)<0))
	{
		P_O_C_R_=C_O_VO_Matrix_*P_C_O_C_L_;
		P_O_C_L_=C_O_VO_Matrix_*P_C_O_C_R_;
	}

	/*	cout<<"P_O_C_L_(1) "<<P_O_C_L_(1)<<" "<<-fabs(P_O_C_L_(1))<<endl;
	P_O_C_L_(1)=-fabs(P_O_C_L_(1));
	P_O_C_L_.Print("P_O_C_L_");*/
	//	P_O_C_L_=P_C_O_C_L_;
	P_U_VO_C_L_=P_VO_+P_O_C_L_;

	S_VO_C_L_(0)=P_U_VO_C_L_(0);	S_VO_C_L_(1)=P_U_VO_C_L_(1);	S_VO_C_L_(2)=P_U_VO_C_L_(2);
	S_VO_C_L_(3)=DP_VO_(0);			S_VO_C_L_(4)=DP_VO_(1);			S_VO_C_L_(5)=DP_VO_(2);


	/*
	P_O_C_R_(1)=fabs(P_O_C_R_(1));
	P_O_C_R_.Print("P_O_C_R_");
	 */
	//	P_O_C_R_=P_C_O_C_R_;
	P_U_VO_C_R_=P_VO_+P_O_C_R_;

	S_VO_C_R_(0)=P_U_VO_C_R_(0);	S_VO_C_R_(1)=P_U_VO_C_R_(1);	S_VO_C_R_(2)=P_U_VO_C_R_(2);
	S_VO_C_R_(3)=DP_VO_(0);			S_VO_C_R_(4)=DP_VO_(1);			S_VO_C_R_(5)=DP_VO_(2);
}

void bimanual_ds::Update()
{
	if (Command==Com_Stop){
		dt_=0;
	} else if (Command==Com_Break)
	{
		dt_=Break_*dt_;
	}
	Frame_transform_U_T_C();

	U_l_=K_l_*(S_VO_C_L_-S_R_L_);
	U_r_=K_r_*(S_VO_C_R_-S_R_R_);

	if (P_O_.Norm2()!=0)
	{
		DGamma_=0.1*(1-Gamma_)/P_O_.Norm2();
	}

	Vector Dummy(6);Dummy.Zero();
	Dummy(3)=S_O_(3);Dummy(4)=S_O_(4);Dummy(5)=S_O_(5);

    // Eq. 9a
	DS_VO_=(A_1_*(S_VO_-S_O_*Gamma_)+DS_O_*Gamma_+S_O_*DGamma_+U_l_+U_r_)/3;

//	DS_VO_=(A_1_*(S_VO_-S_O_*Gamma_)+DS_O_*Gamma_+S_O_*DGamma_+Dummy*DGamma_);

//	cout<<DS_VO_(0)<<" "<<DS_VO_(1)<<" "<<DS_VO_(2)<<" "<<DS_VO_(3)<<" "<<DS_VO_(4)<<" "<<DS_VO_(5)<<endl;



	if (Command==Com_Safe)
	{
		O_U_VO_=O_U_O_*Gamma_O_+O_U_VO_*(1-Gamma_O_);
	}

    S_VO_ = S_VO_ + DS_VO_*dt_;

/*	Pos_Debug.position.x=U_l_(0);
	Pos_Debug.position.y=U_l_(1);
	Pos_Debug.position.z=U_l_(2);
	Pos_Debug.orientation.x=U_l_(3);
	Pos_Debug.orientation.y=U_l_(4);
	Pos_Debug.orientation.z=U_l_(5);
	pub_Debug.publish(Pos_Debug);*/
	//	O_U_VO_=O_U_VO_+DO_VO_*dt_;
	/*	DO_VO_.Print("DO_VO_");*/

	U_O_VO_Matrix_=ConvertQuatsToMats(O_U_VO_);

	C_O_U_Matrix_=U_O_C_Matrix_.Inverse();
	C_O_VO_Matrix_=U_O_VO_Matrix_*C_O_U_Matrix_;


	DS_R_L_=DS_VO_+K_l_*(S_R_L_-S_VO_C_L_);
	S_R_L_=S_R_L_+DS_R_L_*dt_;


	DS_R_R_=DS_VO_+K_r_*(S_R_R_-S_VO_C_R_);
	S_R_R_=S_R_R_+DS_R_R_*dt_;




	Gamma_=Gamma_+DGamma_*dt_*0.2;
	if (Gamma_>0.99) Gamma_=1;




//	if ((P_R_U_L_(2)<0.1)||(P_R_U_R_(2)<0.2))
//	{
//		if (Command!=Com_Stop)
//		{
//			FLag_of_command=0;
//		}
//		Command=Com_Stop;
//		if (FLag_of_command==0)
//		{
//			cout<<"The robot is immediately stopped"<<endl;
//			FLag_of_command=1;
//		}
//	}
	Frame_transform_C_T_U();
	//	cout<<"Norm2() "<<(P_O_-P_VO_).Norm2()+(P_R_L_-P_O_C_L_).Norm2()+(P_R_R_-P_O_C_R_).Norm2()<<endl;
//	if ((((P_O_-P_VO_).Norm2()+(P_R_L_-P_O_C_L_).Norm2()+(P_R_R_-P_O_C_R_).Norm2()<0.01))||P_VO_(0)>0.3)
//	if ((((P_O_-P_VO_).Norm2()+(P_R_L_-P_O_C_L_).Norm2()+(P_R_R_-P_O_C_R_).Norm2()<0.001)))
//	{
	/*	if (Command!=Com_Break)
		{
			FLag_of_command=0;
		}
		Command=Com_Break;
		if (FLag_of_command==0)
		{
			cout<<"The robot's speed is reduced"<<endl;
			FLag_of_command=1;
		}*/
//	}
	//	Gamma_O_=1.0;

    if  (P_U_O_(0)>-0.5)
	{
		Gamma_O_=1.0;
		State_Orie=Per_Follow;
	}

	//Print_states();
}

void bimanual_ds::Frame_transform_C_T_U()
{

	DP_VO_(0)=S_VO_(3);  DP_VO_(1)=S_VO_(4);  DP_VO_(2)=S_VO_(5);
	DP_R_L_(0)=S_R_L_(3);DP_R_L_(1)=S_R_L_(4);DP_R_L_(2)=S_R_L_(5);
	DP_R_R_(0)=S_R_R_(3);DP_R_R_(1)=S_R_R_(4);DP_R_R_(2)=S_R_R_(5);

	P_VO_(0)=S_VO_(0);  P_VO_(1)=S_VO_(1);  P_VO_(2)=S_VO_(2);
	P_R_L_(0)=S_R_L_(0);P_R_L_(1)=S_R_L_(1);P_R_L_(2)=S_R_L_(2);
	P_R_R_(0)=S_R_R_(0);P_R_R_(1)=S_R_R_(1);P_R_R_(2)=S_R_R_(2);

	DDP_VO_(0)=DS_VO_(3); DDP_VO_(1)=DS_VO_(4); DDP_VO_(2)=DS_VO_(5);
	DDP_R_L_(0)=DS_R_L_(3);DDP_R_L_(1)=DS_R_L_(4);DDP_R_L_(2)=DS_R_L_(5);
	DDP_R_R_(0)=DS_R_R_(3);DDP_R_R_(1)=DS_R_R_(4);DDP_R_R_(2)=DS_R_R_(5);

	P_R_U_L_=P_R_L_+P_O_C_;
	P_R_U_R_=P_R_R_+P_O_C_;
}

void bimanual_ds::initialize_Virrtual_object()
{
	Frame_transform_U_T_C();
	P_VO_=P_R_L_+P_R_R_;
	O_U_VO_=O_U_O_;
	U_O_C_Matrix_=ConvertQuatsToMats(O_U_VO_);
	P_VO_.Mult(0.5,P_VO_);
	DP_VO_.Zero();
	DDP_VO_.Zero();
	DO_VO_.Zero();
}

void bimanual_ds::Debug(Matrix & _Data_Debug)
{

	_Data_Debug.Resize(3,9);
	_Data_Debug.Zero();

	_Data_Debug.SetColumn(P_VO_+P_O_C_,0);// VIrtual Object Position
	_Data_Debug.SetColumn(P_U_O_,1);// Object Position
	_Data_Debug.SetColumn(P_U_VO_C_L_+P_O_C_,2);// The left Catching Position on Virtual Object
	_Data_Debug.SetColumn(P_R_U_L_,3);// The left robot Position
	_Data_Debug.SetColumn(P_U_O_C_L_,4);// The left Catching Position on Object
	_Data_Debug.SetColumn(P_U_VO_C_R_+P_O_C_,5);// The right Catching Position on Virtual Object
	_Data_Debug.SetColumn(P_R_U_R_,6);// The right robot Position
	_Data_Debug.SetColumn(P_U_O_C_R_,7);// The right Catching Position on Object

	//	_Data_Debug.Print("_Data_Debug");
}

ENUM_State	bimanual_ds::What_is_the_state()
{
	return Command;
}

Vector bimanual_ds::Get_virtual_object_pos()
{

	return P_VO_+P_O_C_;
}


Vector bimanual_ds::Get_virtual_object_orie()
{

	return O_U_VO_;
}

bool bimanual_ds::What_is_the_Orie_state()
{
	if (State_Orie==Not_Follow)
	{
		return false;
	}
	else
	{
		return true;
	}
}
double bimanual_ds::Getdt_(){
	return dt_;
}

double bimanual_ds::GetGamma_(){
	return Gamma_;
}
