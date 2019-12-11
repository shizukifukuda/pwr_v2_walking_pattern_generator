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

Vector3d sub_real_CP;
void Sub_real_CP_Callback(geometry_msgs::PointStamped& real_CP){
	sub_real_CP[0] = real_CP.point.x;
	sub_real_CP[1] = real_CP.point.y;
	sub_real_CP[2] = real_CP.point.z;
}

Vector3d desired_ZMP(Vector3d P_ref, Vector3d CP_real, Vector3d CP_ref, double K, double omega){
	Vector3d P_des;
	P_des = P_ref + (1 + (K/omega))*(CP_real - CP_ref);
	return P_des;
}
Vector3d refference_CP_Orbit_ti(Vector3d P_ref, Vector3d CP_ref, double time, double omega){
	Vector3d CP_ref_ti;
	CP_ref_ti = P_ref + exp(omega*time) * (CP_ref - P_ref);
	return CP_ref_ti;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "pwr_v2_walking_pattern_generator_node");
	ros::NodeHandle nh;

	ros::Subscriber real_CP_position_sub = nh.subscribe("pwr_v2_Capture_point",10,Sub_real_CP_Callback);

	ros::Publisher leg_mode_pub = nh.advertise< std_msgs::Float64 >("leg_mode",10);
	ros::Publisher left_foot_position_pub = nh.advertise< geometry_msgs::PointStamped >("left_foot_point",10);
	ros::Publisher right_foot_position_pub = nh.advertise< geometry_msgs::PointStamped >("right_foot_point",10);

	double K = 5.0;
	double omega = sqrt(G/(CoM[2] - ZMP[2]));

	

	return 0;
}