#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>
// 行列演算ライブラリ
#include "Eigen/Core"
#include "Eigen/Geometry"
#define EIGEN_NO_DEBUG // コード内のassertを無効化．
#define EIGEN_DONT_VECTORIZE // SIMDを無効化．
#define EIGEN_DONT_PARALLELIZE // 並列を無効化．
#define EIGEN_MPL2_ONLY // LGPLライセンスのコードを使わない．
using namespace Eigen;

#define G 9.805
#define K 5.0
#define CoM_HEIGHT 0.280

Vector3d sub_real_CP;
void Sub_real_CP_Callback(geometry_msgs::PointStamped& real_CP){
	sub_real_CP[0] = real_CP.point.x;
	sub_real_CP[1] = real_CP.point.y;
	sub_real_CP[2] = real_CP.point.z;
}

Vector3d desired_ZMP(Vector3d P_ref, Vector3d CP_real, Vector3d CP_ref, double omega){
	Vector3d P_des;
	P_des = P_ref + (1 + (K/omega))*(CP_real - CP_ref);
	return P_des;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "pwr_v2_walking_pattern_generator_node");
	ros::NodeHandle nh;

	ros::Subscriber real_CP_position_sub = nh.subscribe("pwr_v2_Capture_point",10,Sub_real_CP_Callback);

	ros::Publisher leg_mode_pub = nh.advertise< std_msgs::Float64 >("leg_mode",10);
	ros::Publisher left_foot_position_pub = nh.advertise< geometry_msgs::PointStamped >("left_foot_point",10);
	ros::Publisher right_foot_position_pub = nh.advertise< geometry_msgs::PointStamped >("right_foot_point",10);

	int step = 5;
	double limit_time = 1.0;	//一歩あたりの計算時間
	int division = 100;	//分割数
	double Hz = 1 / (limit_time / (double)division);	//制御周期
	double time;
	int i;
	MatrixXd CoM = MatrixXd::Zero(3,step);
	for (i = 0; i < step; i++ )CoM(2,i) = CoM_HEIGHT;

	// 基準ZMPの設定
	MatrixXd ZMP_ref(3,step);
	ZMP_ref <<  0.0, 0.05, 0.10,0.15, 0.20,
				0.0, 0.01,-0.01,0.01,-0.01,
				0.0, 0.00, 0.00,0.00, 0.00;

	// 基準CPの設定
	MatrixXd CP_ref_ini(3,step);
	MatrixXd CP_ref_end(3,step);
	double num_time = limit_time * step;
	double omega = sqrt(G / ( CoM(2,0) - ZMP_ref(2,0) ) );
	CP_ref_end.col(step-1) = ZMP_ref.col(step-1);
	for(i = step-1 ; i >= 1; i--){
		CP_ref_ini.col(i) = ZMP_ref.col(i) + exp(-omega*num_time) * (CP_ref_end.col(i) - ZMP_ref.col(i));
		CP_ref_end.col(i-1) = CP_ref_ini.col(i);
		num_time = num_time - limit_time;
	}
	CP_ref_ini.col(0) = ZMP_ref.col(0) + exp(-omega*num_time) * (CP_ref_ini.col(0) - ZMP_ref.col(0));

	// 基準CP軌道の設計
	int data_count = step * (int)Hz;
	MatrixXd CP_ref_ti(3,data_count);
	int s = 0;
	ros::Rate loop_rate(Hz);
	time=ros::Time::now().toSec;
	for(i = 0 ; i <= step ; i++){
		for (int j=0 ; j<division ; j++){
			time = ros::Time::now().toSec - time;
			CP_ref_ti.col(s) = ZMP_ref.col(i) + exp(omega*time) * (CP_ref_ini.col(i) - ZMP_ref.col(i));
			s++;
			loop_rate.sleep();
		}
	}

	// CPの誤差を修正するZMPの計算と遊脚の目標値を生成
	MatrixXd ZMP_des = MatrixXd::Zero(3,data_count);
	i = 0;
	s = 1;
	while(ros::ok()){
		//CPの誤差を修正する所望のZMP
		ZMP_des.col(i) = desired_ZMP(ZMP_ref.col(s-1),sub_real_CP,CP_ref_ti.col(i),omega);
		//基準ZMPと間をつなぐ遊脚の足先軌道を生成

		i++;
		if(s % division == 0)s++;
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}