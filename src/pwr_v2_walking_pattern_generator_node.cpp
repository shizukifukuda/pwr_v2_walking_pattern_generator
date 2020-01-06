#include <stdio.h>
#include <math.h>
#include <fstream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>


// 行列演算ライブラリ
// #define EIGEN_NO_DEBUG // コード内のassertを無効化．
// #define EIGEN_DONT_VECTORIZE // SIMDを無効化．
// #define EIGEN_DONT_PARALLELIZE // 並列を無効化．
// #define EIGEN_MPL2_ONLY // LGPLライセンスのコードを使わない．
#include "Eigen/Core"
#include "Eigen/Dense"
#define PRINT_MAT(X) cout << #X << ":\n" << X << endl << endl
#define PRINT_MAT2(X,DESC) cout << DESC << ":\n" << X << endl << endl
#define PRINT_FNC    cout << "[" << __func__ << "]" << endl

using namespace Eigen;

#define G 9.805					// 重力加速度
#define K 5.0						// ゲイン
#define CoM_HEIGHT 0.230			// 重心の高さ
#define step  8						// 基準ZMPの数
#define division 100				// 一歩あたりの分割数
#define limit_time 2.0				// 一歩あたりの計算時間（sec）

Vector3d Left_foot_init_position(0.0, 0.033, 0.0);		//左足先の初期位置
Vector3d Right_foot_init_position(0.0, -0.033, 0.0);	//右足先の初期位置

Vector3d sub_real_CP;
void Sub_real_CP_Callback(const geometry_msgs::PointStamped &real_CP)
{
	sub_real_CP(0) = real_CP.point.x;
	sub_real_CP(1) = real_CP.point.y;
	sub_real_CP(2) = real_CP.point.z;
}

geometry_msgs::PointStamped setPoint(Vector3d position, int seq, ros::Time stamp_time)
{
	geometry_msgs::PointStamped ps;
		ps.header.seq = seq;
		ps.header.stamp = stamp_time;
		ps.point.x = position(0);
		ps.point.y = position(1);
		ps.point.z = position(2);
	return ps;
}

void RungeKutta(MatrixXd dX, MatrixXd &X, MatrixXd u, double tt, double dt, MatrixXd A, MatrixXd B, MatrixXd C, MatrixXd D) {
    MatrixXd k1 = A*X + B*u;
    MatrixXd k2 = A*(X + 0.5*k1*dt) + B*u;
    MatrixXd k3 = A*(X + 0.5*k2*dt) + B*u;
    MatrixXd k4 = A*(X + k3*dt) + B*u;
    MatrixXd k = (k1 + 2.0*k2 + 2.0*k3 + k4)*dt / 6.0;
    X = X + k;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "pwr_v2_walking_pattern_generator_node");
	ros::NodeHandle nh;

	// ros::Subscriber real_CP_position_sub = nh.subscribe("pwr_v2_Capture_point",10,Sub_real_CP_Callback);

	ros::Publisher leg_mode_pub = nh.advertise<std_msgs::Float64>("leg_mode",10);
	ros::Publisher left_foot_position_pub = nh.advertise<geometry_msgs::PointStamped>("left_foot_point",10);
	ros::Publisher right_foot_position_pub = nh.advertise<geometry_msgs::PointStamped>("right_foot_point",10);
	ros::Publisher LIP_CoM_position_pub = nh.advertise<geometry_msgs::PointStamped>("LIP_CoM_point",10);
	
	// 基準ZMPの設定
	MatrixXd ZMP_ref(3,step);
	ZMP_ref <<  0.00, 0.00, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60,
				0.00,-0.03, 0.03,-0.03, 0.03,-0.03, 0.03,-0.03,
				0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00;

	double omega = sqrt(G / CoM_HEIGHT);		// 角速度
	double Hz = 1 / ((double)limit_time / (double)division);		// ループ周期
	ROS_INFO("Hz = %lf",Hz);

	// 基準CPの設定
	MatrixXd CP_ref_ini(3,step);
	MatrixXd CP_ref_end(3,step);
	double num_time = (double)limit_time * (double)(step-1);
	int i;
	CP_ref_end.col(step-1) = ZMP_ref.col(step-1);
	for(i = step-1 ; i > 0 ; i--){
		CP_ref_ini.col(i) = ZMP_ref.col(i) + exp(-1.0*omega*num_time) * (CP_ref_end.col(i) - ZMP_ref.col(i));
		CP_ref_end.col(i-1) = CP_ref_ini.col(i);
		num_time = num_time - (double)limit_time;
		// std::cout << "CP_ref_ini = \n"<< CP_ref_ini.col(i) << std::endl;
		// std::cout << "num_time = "<< num_time << std::endl;
	}
	CP_ref_ini.col(0) = ZMP_ref.col(0) + exp(-1*omega*num_time) * (CP_ref_ini.col(0) - ZMP_ref.col(0));
	ROS_INFO("[WPG] Set Reference Capture Point");
	// std::cout << "CP_ref_ini = \n"<< CP_ref_ini.col(0) << std::endl;
	// std::cout << "num_time = "<< num_time << std::endl;

	// 基準CP軌道の設計
	double base_time = ros::Time::now().toSec();
	double time = 0.0;
	int data_count = (step-1) * (int)division;
	MatrixXd CP_ref_ti(3,data_count);
	ROS_INFO("[WPG] data_count = %d",data_count);
	int s = 0;
	int j;
	ros::Rate loop_rate(Hz);
	for(i=0 ; i<(step-1) ; i++){
		for (j=0 ; j<division ; j++){
			CP_ref_ti.col(s) = ZMP_ref.col(i) + (exp(omega*time) * (CP_ref_ini.col(i) - ZMP_ref.col(i)));
			loop_rate.sleep();
			double time = ros::Time::now().toSec() - base_time;
			// ROS_INFO("[debag] %d-%d,(%d)",i,j,s);
			// std::cout << "CP_ref_ti = \n"<< CP_ref_ti.col(s) << std::endl;
			s++;
		}
	}
	ROS_INFO("[WPG] Set Reference Capture Point Load");

	// 基準ZMP間をつなぐ遊脚の足先軌道を生成
	MatrixXd Free_Leg_position = MatrixXd::Zero(3,data_count);
	Free_Leg_position.col(0) = Left_foot_init_position; //左足先の初期位置
	// 最初の一歩目
	j = 1;
	Vector3d start_position = Free_Leg_position.col(0);
	Vector3d end_position = ZMP_ref.col(2);
	double dx = (end_position(0) - start_position(0)) / (2*division);
	double dy = (end_position(1) - start_position(1)) / (2*division);
	double dz = 0.01/division;
	for(i=1 ; i<(2*division) ; i++){
		Free_Leg_position(0,j) += dx;
		Free_Leg_position(1,j) += dy;
		if( i<division ) Free_Leg_position(2,j) += dz;
		else if( i>=division ) Free_Leg_position(2,j) -= dz;
		j++;
	}
	ROS_INFO("[WPG] Set Free Leg Load (1)");
	// 2歩目以降
	for (s=3 ; s<step ; s++){
		start_position = ZMP_ref.col(s-2);
		end_position = ZMP_ref.col(s);
		dx = (end_position(0) - start_position(0)) / division;
		dy = (end_position(1) - start_position(1)) / division;
		dz = 0.01 / (division/2);
		for(i=1 ; i<division ; i++){
			Free_Leg_position(0,j) += dx;
			Free_Leg_position(1,j) += dy;
			if( i<(division/2) ) Free_Leg_position(2,j) += dz;
			else if( i>=(division/2) ) Free_Leg_position(2,j) -= dz;
			j++;
		}
	}
	ROS_INFO("[WPG] Set Free Leg Load (2~)");
	
	// CPの誤差を修正するZMPの計算と左右脚の目標を配信
	MatrixXd ZMP_des = MatrixXd::Zero(3,data_count);
	MatrixXd LIP_CoM_pos = MatrixXd::Zero(3,data_count);
	MatrixXd LIP_CoM_spd = MatrixXd::Zero(3,data_count);
	MatrixXd LIP_CP  = MatrixXd::Zero(3,data_count);
	geometry_msgs::PointStamped right_foot_msg;
	geometry_msgs::PointStamped left_foot_msg;
	geometry_msgs::PointStamped waist_msg;
	std_msgs::Float64 leg_mode_msg;
	leg_mode_msg.data = 1.0;
	leg_mode_pub.publish(leg_mode_msg);

	MatrixXd A(2, 2);
    A(0, 0) = 0;
    A(0, 1) = 1;
    A(1, 0) = pow(omega,2);
    A(1, 1) = 0;
    MatrixXd B(2, 1);
    B(0, 0) = 0;
    B(1, 0) = -1*pow(omega,2);

    MatrixXd C(2, 2);
    C(0, 0) = 1;
    C(0, 1) = 0;
	C(1, 0) = 0;
	C(1, 1) = 1;
    MatrixXd D(2, 1);
    D(0, 0) = 0;
	D(1, 0) = 0;

    double tt = 0.0;
	double dt = (double)limit_time/(double)division;
    MatrixXd X(2, 1);
    X(0, 0) = 0;
    X(1, 0) = 0;
    MatrixXd dX(2, 1);
    dX(0, 0) = 0;
    dX(1, 0) = 0;
    MatrixXd u(1, 1);
    u(0, 0) = 0;
    MatrixXd Yx(2, 1);
    Yx(0, 0) = 0;
	Yx(1, 0) = 0;
	MatrixXd Yy(2, 1);
    Yy(0, 0) = 0;
	Yy(1, 0) = 0;

	std::ofstream fout("/home/shizuki/SynologyDrive/大学/修士論文/record/check.csv");
	fout << "UNIX_time[sec],CP_ref_x,CP_ref_y,ZMP_des_x,ZMP_des_y,CoM_position_x,CoM_positon_y,CP_lip_x,CP_lip_y" << std::endl;

	s = 0;
	i = 1;
	while(ros::ok() && i<data_count){
		ros::Time TIME = ros::Time::now();

		u(0,0) = ZMP_des(0,i-1);
		RungeKutta(dX, X, u, tt, dt, A, B, C, D);
        Yx = C*X;
		LIP_CoM_pos(0,i-1) = Yx(0);
		LIP_CoM_spd(0,1-1) = Yx(1);
		LIP_CP(0,i-1) = Yx(0) + (Yx(1)/omega);

		u(0,0) = ZMP_des(1,i-1);
		RungeKutta(dX, X, u, tt, dt, A, B, C, D);
        Yy = C*X;
		LIP_CoM_pos(1,i-1) = Yy(0);
		LIP_CoM_spd(1,i-1) = Yy(1);
		LIP_CP(1,i-1) = Yy(0) + (Yy(1)/omega);

		LIP_CoM_pos(2,i-1) = CoM_HEIGHT;

		// CPの誤差を修正する所望のZMP(bno055使用)
		// ZMP_des.col(i) = ZMP_ref.col(s) + ( (1.0 + (K/omega)) * ( sub_real_CP - CP_ref_ti.col(i-1) ) );

		// CPの誤差を修正する所望のZMP(LIP model 使用)
		ZMP_des.col(i) = ZMP_ref.col(s) + ( ( 1.0 + ( (double)K / omega ) ) * ( LIP_CP.col(i-1) - CP_ref_ti.col(i-1) ) );

		std::cout << "ZMP_des = \n"<< ZMP_des.col(i-1) << std::endl;
		std::cout << "ZMP_ref = \n"<< ZMP_ref.col(s) << std::endl;
		std::cout << "LIP_CoM_pos = \n"<< LIP_CoM_pos.col(i-1) << std::endl;
		std::cout << "LIP_CP = \n"<< LIP_CP.col(i-1) << std::endl;
		std::cout << "CP_ref_ti = \n"<< CP_ref_ti.col(i-1) << std::endl;
		
		if(i<=division){
			right_foot_msg = setPoint(Right_foot_init_position ,i,TIME);
			left_foot_msg = setPoint(Free_Leg_position.col(i-1),i,TIME);
		}
		// Sup:Right / Free:Left
		else if(leg_mode_msg.data==1.0 && i>division){
			right_foot_msg = setPoint(ZMP_des.col(i-1),i,TIME);
			left_foot_msg = setPoint(Free_Leg_position.col(i-1),i,TIME);
		}
		// Sup:Left / Free:Right
		else if(leg_mode_msg.data==-1.0 && i>division){
			right_foot_msg = setPoint(Free_Leg_position.col(i-1),i,TIME);
			left_foot_msg = setPoint(ZMP_des.col(i-1),i,TIME);
		}
		if(i%division ==0 && i<data_count){
			s++;
			if(i>=(2*division)){
				leg_mode_msg.data = leg_mode_msg.data * -1.0;
				leg_mode_pub.publish(leg_mode_msg);
			}
		}
		waist_msg = setPoint(LIP_CoM_pos.col(i-1),i,TIME);
		LIP_CoM_position_pub.publish(waist_msg);
		right_foot_position_pub.publish(right_foot_msg);
		left_foot_position_pub.publish(left_foot_msg);
		ROS_INFO("[WPG] Publish Point (i=%d, leg_mode=%1.1f)",i,leg_mode_msg.data);
		double time = TIME.toSec();
		fout << time <<","<< 
		CP_ref_ti(0,i-1) <<","<< CP_ref_ti(1,i-1) <<","<<
		ZMP_des(0,i-1) <<","<< ZMP_des(1,i-1) <<","<< 
		LIP_CoM_pos(0,i-1) <<","<< LIP_CoM_pos(1,i-1) <<","<<
		LIP_CP(0,i-1) <<","<< LIP_CP(1,i-1) <<","<<
		std::endl;

		loop_rate.sleep();
		ros::spinOnce();
		i++;
	}
	return 0;
}