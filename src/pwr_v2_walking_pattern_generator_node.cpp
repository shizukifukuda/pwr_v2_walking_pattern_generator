#include <stdio.h>
#include <math.h>
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
#define step  5						// 歩数
#define division 60				// 一歩あたりの分割数
#define limit_time 1				// 一歩あたりの計算時間（sec）

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
	ZMP_ref <<  0.00, 0.00, 0.10, 0.20, 0.30,
				0.00,-0.033, 0.033,-0.033, 0.033,
				0.00, 0.00, 0.00, 0.00, 0.00;

	double omega = sqrt(G / ( CoM_HEIGHT - ZMP_ref(2,0) ) );	// 角速度
	double Hz = 1 / (limit_time / (double)division);			// ループ周期
	ROS_INFO("Hz = %lf",Hz);

	// 基準CPの設定
	MatrixXd CP_ref_ini(3,step);
	MatrixXd CP_ref_end(3,step);
	double num_time = limit_time * step;
	int i;
	CP_ref_end.col(step-1) = ZMP_ref.col(step-1);
	for(i = step-1 ; i >= 1; i--){
		CP_ref_ini.col(i) = ZMP_ref.col(i) + exp(-1*omega*num_time) * (CP_ref_end.col(i) - ZMP_ref.col(i));
		CP_ref_end.col(i-1) = CP_ref_ini.col(i);
		num_time = num_time - limit_time;
		// std::cout << "CP_ref_ini = \n"<< CP_ref_ini.col(i) << std::endl;
		// std::cout << "num_time = "<< num_time << std::endl;
	}
	CP_ref_ini.col(0) = ZMP_ref.col(0) + exp(-1*omega*num_time) * (CP_ref_ini.col(0) - ZMP_ref.col(0));
	ROS_INFO("[WPG] Set Reference Capture Point");
	// std::cout << "CP_ref_ini = \n"<< CP_ref_ini.col(0) << std::endl;
	// std::cout << "num_time = "<< num_time << std::endl;

	// 基準CP軌道の設計
	double base_time = ros::Time::now().toSec();
	int data_count = (step-1) * (int)division;
	MatrixXd CP_ref_ti(3,data_count);
	ROS_INFO("[WPG] data_count = %d",data_count);
	int s = 0;
	int j;
	ros::Rate loop_rate(Hz);
	for(i=0 ; i<(step-1) ; i++){
		for (j=0 ; j<division ; j++){
			loop_rate.sleep();
			double time = ros::Time::now().toSec() - base_time;
			CP_ref_ti.col(s) = ZMP_ref.col(i) + (exp(omega*time) * (CP_ref_ini.col(i) - ZMP_ref.col(i)));
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
	s = 2;
	j = 1;
	Vector3d start_position = Free_Leg_position.col(s-2);
	Vector3d end_position = ZMP_ref.col(s);
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
	double dt = (double)limit_time / (double)division;
	MatrixXd dT(1,1);
	dT(0,0) = dt;
	std::cout << "dT = " << dT << std::endl;
	MatrixXd OMEGA = MatrixXd::Constant(1,1,omega);
	s = 0;
	i = 1;
	leg_mode_msg.data = 1.0;
	leg_mode_pub.publish(leg_mode_msg);

	while(ros::ok() && i<data_count){
		// CPの誤差を修正する所望のZMP(bno055使用)
		// ROS_INFO("ZMP_des");
		// ZMP_des.col(i-1) = ZMP_ref.col(s) + ( (1.0 + (K/omega)) * ( sub_real_CP - CP_ref_ti.col(i-1) ) );
		// CPの誤差を修正する所望のZMP(LIP model 使用)
		ROS_INFO("ZMP_des");
		ZMP_des.col(i-1) = ZMP_ref.col(s) + ( ( 1.0 + (K/omega) ) * ( LIP_CP.col(i-1) - CP_ref_ti.col(i-1) ) );
		std::cout << "ZMP_des = \n"<< ZMP_des.col(i-1) << std::endl;
		std::cout << "ZMP_ref = \n"<< ZMP_ref.col(s) << std::endl;
		// CPを求める
		LIP_CoM_spd.col(i) = OMEGA(0,0) * (CP_ref_ti.col(i-1) - LIP_CoM_pos.col(i-1));
		LIP_CoM_pos(0,i) = LIP_CoM_pos(0,i-1) + ( LIP_CoM_spd(0,i)*dT(0,0) );
		LIP_CoM_pos(1,i) = LIP_CoM_pos(1,i-1) + ( LIP_CoM_spd(1,i)*dT(0,0) );
		LIP_CoM_pos(2,i) = CoM_HEIGHT;
		LIP_CP.col(i) = LIP_CoM_pos.col(i) + ( LIP_CoM_spd.col(i) / OMEGA(0,0) );
		std::cout << "LIP_CoM_spd = \n"<< LIP_CoM_spd.col(i) << std::endl;
		std::cout << "LIP_CoM_pos = "<< std::endl;
		std::cout << LIP_CoM_pos(0,i) << std::endl;
		std::cout << LIP_CoM_pos(1,i) << std::endl;
		std::cout << LIP_CoM_pos(2,i) << std::endl;
		std::cout << "LIP_CP = \n"<< LIP_CP.col(i) << std::endl;
		std::cout << "CP_ref_ti = \n"<< CP_ref_ti.col(i-1) << std::endl;
		
		ros::Time TIME = ros::Time::now();
		if(i<division){
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
		waist_msg = setPoint(LIP_CoM_pos.col(i),i,TIME);
		LIP_CoM_position_pub.publish(waist_msg);
		right_foot_position_pub.publish(right_foot_msg);
		left_foot_position_pub.publish(left_foot_msg);
		ROS_INFO("[WPG] Publish Point (i=%d, leg_mode=%1.1f)",i,leg_mode_msg.data);

		loop_rate.sleep();
		ros::spinOnce();
		i++;
	}
	return 0;
}