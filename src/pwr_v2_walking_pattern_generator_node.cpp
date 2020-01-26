#include <stdio.h>
#include <math.h>
#include <fstream>
#include <ros/ros.h>
#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <sensor_msgs/JointState.h>
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

#define _USE_MATH_DEFINES
#define G 9.805					// 重力加速度
int step = 7;					// 基準ZMPの数
int division = 100;				// 一歩あたりの分割数
double limit_time = 1.0;		// 一歩あたりの計算時間（sec）

Vector3d Left_foot_init_position(0.0, 0.033, 0.0);		//左足先の初期位置
Vector3d Right_foot_init_position(0.0, -0.033, 0.0);	//右足先の初期位置

sensor_msgs::JointState joint_state_publisher(float *joint_state, ros::Time time){
	sensor_msgs::JointState js;
	js.header.stamp = time;
	js.name.resize(10);
	js.name[0] = "Left_joint_Ankle_roll";
	js.name[1] = "Left_joint_Ankle_pitch";
	js.name[2] = "Left_joint_knee";
	js.name[3] = "Left_joint_hipjoint_pitch";
	js.name[4] = "Left_joint_hipjoint_roll";
	js.name[5] = "Right_joint_Ankle_roll";
	js.name[6] = "Right_joint_Ankle_pitch";
	js.name[7] = "Right_joint_knee";
	js.name[8] = "Right_joint_hipjoint_pitch";
	js.name[9] = "Right_joint_hipjoint_roll";
	js.position.resize(10);
	js.position[0] = joint_state[0];
	js.position[1] = joint_state[1];
	js.position[2] = joint_state[2];
	js.position[3] = joint_state[3];
	js.position[4] = joint_state[4];
	js.position[5] = joint_state[5];
	js.position[6] = joint_state[6];
	js.position[7] = joint_state[7];
	js.position[8] = joint_state[8];
	js.position[9] = joint_state[9];
	return js;
}

Vector3d sub_CoM_vector;
geometry_msgs::PoseStamped sub_CoM_pose;
void Sub_real_CoM_Callback(const geometry_msgs::PoseStamped &real_CoM){
	sub_CoM_vector(0) = real_CoM.pose.position.x;
	sub_CoM_vector(1) = real_CoM.pose.position.y;
	sub_CoM_vector(2) = real_CoM.pose.position.z;
	sub_CoM_pose = real_CoM;
}
// Low Path Filter
Vector3d LP_Filter(double rc, double ts, Vector3d f_x1, Vector3d f_x2){
	Vector3d LP_fx;
	double alpha = (1/rc) / ((1/ts) + (1/rc));
	return LP_fx = alpha * f_x2 + (1 - alpha) * f_x1;
}

geometry_msgs::PointStamped setPoint(Vector3d position, int seq, ros::Time stamp_time, std::string name){
	geometry_msgs::PointStamped ps;
		ps.header.seq = seq;
		ps.header.stamp = stamp_time;
		ps.header.frame_id = name;
		ps.point.x = position(0);
		ps.point.y = position(1);
		ps.point.z = position(2);
	return ps;
}

void waist_pose_publisher(Vector3d CoM, ros::Time time, int mode){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(CoM[0], CoM[1], CoM[2]) );
	tf::Quaternion q;
	if(mode == 1||mode == 3)q.setRPY(0, 0, 0);
	if(mode == 2)q = tf::Quaternion(-sub_CoM_pose.pose.orientation.y, sub_CoM_pose.pose.orientation.x, sub_CoM_pose.pose.orientation.z, sub_CoM_pose.pose.orientation.w);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, time, "world", "base_link"));
}

MatrixXd RungeKutta(MatrixXd dX, MatrixXd X, MatrixXd u, double tt, double dt, MatrixXd A, MatrixXd B, MatrixXd C, MatrixXd D) {
    MatrixXd k1 = A*X + B*u;
    MatrixXd k2 = A*(X + 0.5*k1*dt) + B*u;
    MatrixXd k3 = A*(X + 0.5*k2*dt) + B*u;
    MatrixXd k4 = A*(X + k3*dt) + B*u;
    MatrixXd k = (k1 + 2.0*k2 + 2.0*k3 + k4)*dt / 6.0;
    return X = X + k;
}

float joint_angle[10];
void IK_solver(ros::NodeHandle& nh, std::string select_leg, Vector3d position, std::string urdf_param, double timeout, double eps, int mode){
	KDL::Chain chain;
	KDL::JntArray ll, ul; //lower joint limits(下限関節角), upper joint limits(上限間節角)
	bool valid;

	//エンドエフェクタの設定
	int rc;
	double EuZ=0.0, EuY=0.0, EuX=0.0;
	KDL::Rotation eep_Rotation;
	eep_Rotation = eep_Rotation.EulerZYX(EuZ,EuY,EuX); //回転行列
	if(mode==2 && select_leg=="s_left")eep_Rotation = eep_Rotation.Quaternion(-sub_CoM_pose.pose.orientation.y, sub_CoM_pose.pose.orientation.x, sub_CoM_pose.pose.orientation.z, sub_CoM_pose.pose.orientation.w);
	if(mode==2 && select_leg=="s_right")eep_Rotation = eep_Rotation.Quaternion(-sub_CoM_pose.pose.orientation.y, sub_CoM_pose.pose.orientation.x, sub_CoM_pose.pose.orientation.z, sub_CoM_pose.pose.orientation.w);
	KDL::Vector eep_Vector; //位置ベクトル
	eep_Vector.x(position(0));  //位置の目標値を各要素に代入
	eep_Vector.y(position(1));
	eep_Vector.z(position(2));
	KDL::Frame end_effector_pose(eep_Rotation,eep_Vector); //目標の姿勢行列
	KDL::JntArray result; //逆運動学により求まる関節角の計算結果

	if(select_leg == "s_left"){
		ROS_INFO("[IK]s_left_ik_solver");
		TRAC_IK::TRAC_IK s_left_IK_solver("Left_link_foot", "waist", urdf_param, timeout, eps);
		valid = s_left_IK_solver.getKDLChain(chain);
		if (!valid) {
			ROS_ERROR("[IK]There was no valid KDL chain found");//有効なKDLチェーンが見つかりませんでした
			return;
		}
		valid = s_left_IK_solver.getKDLLimits(ll,ul);
		if (!valid) {
			ROS_ERROR("[IK]There were no valid KDL joint limits found");//有効なKDL関節限度が見つかりませんでした
			return;
		}
		//関節の限度が問題ないかのチェック
		assert(chain.getNrOfJoints() == ll.data.size());
		assert(chain.getNrOfJoints() == ul.data.size());
		// すべての関節限界の中間に名目上のチェーン構成を作成する
		KDL::JntArray s_left_nominal(chain.getNrOfJoints());
		for (uint j=0; j<s_left_nominal.data.size(); j++) {
			s_left_nominal(j) = (ll(j)+ul(j))/2.0;
		}
		rc = s_left_IK_solver.CartToJnt(s_left_nominal,end_effector_pose,result);
		if(rc>=0){
			ROS_INFO("[IK]%s IK is Success!",select_leg.c_str());
		}else{
			ROS_ERROR("[IK]%s IK is Error (rc = %d)",select_leg.c_str(),rc);
			ROS_ERROR("[IK]Calculation is not possible with that target value");
			return;
		}
		joint_angle[0] = result.operator()(0,0);
		joint_angle[1] = result.operator()(1,0);
		joint_angle[2] = result.operator()(2,0);
		joint_angle[3] = result.operator()(3,0);
		joint_angle[4] = result.operator()(4,0);
	}else if(select_leg == "f_right"){
		ROS_INFO("[IK]f_right_ik_solver");
		TRAC_IK::TRAC_IK f_right_IK_solver("waist", "Right_link_foot", urdf_param, timeout, eps);
		valid = f_right_IK_solver.getKDLChain(chain);
		if (!valid) {
			ROS_ERROR("[IK]There was no valid KDL chain found");//有効なKDLチェーンが見つかりませんでした
			return;
		}
		valid = f_right_IK_solver.getKDLLimits(ll,ul);
		if (!valid) {
			ROS_ERROR("[IK]There were no valid KDL joint limits found");//有効なKDL関節限度が見つかりませんでした
			return;
		}
		//関節の限度が問題ないかのチェック
		assert(chain.getNrOfJoints() == ll.data.size());
		assert(chain.getNrOfJoints() == ul.data.size());
		// すべての関節限界の中間に名目上のチェーン構成を作成する
		KDL::JntArray f_right_nominal(chain.getNrOfJoints());
		for (uint j=0; j<f_right_nominal.data.size(); j++) {
			f_right_nominal(j) = (ll(j)+ul(j))/2.0;
		}
		rc = f_right_IK_solver.CartToJnt(f_right_nominal,end_effector_pose,result);
		if(rc>=0){
			ROS_INFO("[IK]%s IK is Success!",select_leg.c_str());
		}else{
			ROS_ERROR("[IK]%s IK is Error (rc = %d)",select_leg.c_str(),rc);
			ROS_ERROR("[IK]Calculation is not possible with that target value");
			return;
		}
		joint_angle[9] = result.operator()(0,0);
		joint_angle[8] = result.operator()(1,0);
		joint_angle[7] = result.operator()(2,0);
		joint_angle[6] = result.operator()(3,0);
		joint_angle[5] = result.operator()(4,0);
	}else if(select_leg == "f_left"){
		ROS_INFO("[IK]f_left_ik_solver");
		TRAC_IK::TRAC_IK f_left_IK_solver("waist", "Left_link_foot", urdf_param, timeout, eps);
		valid = f_left_IK_solver.getKDLChain(chain);
		if (!valid) {
			ROS_ERROR("[IK]There was no valid KDL chain found");//有効なKDLチェーンが見つかりませんでした
			return;
		}
		valid = f_left_IK_solver.getKDLLimits(ll,ul);
		if (!valid) {
			ROS_ERROR("[IK]There were no valid KDL joint limits found");//有効なKDL関節限度が見つかりませんでした
			return;
		}
		//関節の限度が問題ないかのチェック
		assert(chain.getNrOfJoints() == ll.data.size());
		assert(chain.getNrOfJoints() == ul.data.size());
		// すべての関節限界の中間に名目上のチェーン構成を作成する
		KDL::JntArray f_left_nominal(chain.getNrOfJoints());
		for (uint j=0; j<f_left_nominal.data.size(); j++) {
			f_left_nominal(j) = (ll(j)+ul(j))/2.0;
		}
		rc = f_left_IK_solver.CartToJnt(f_left_nominal,end_effector_pose,result);
		if(rc>=0){
			ROS_INFO("[IK]%s IK is Success!",select_leg.c_str());
		}else{
			ROS_ERROR("[IK]%s IK is Error (rc = %d)",select_leg.c_str(),rc);
			ROS_ERROR("[IK]Calculation is not possible with that target value");
			return;
		}
		joint_angle[4] = result.operator()(0,0);
		joint_angle[3] = result.operator()(1,0);
		joint_angle[2] = result.operator()(2,0);
		joint_angle[1] = result.operator()(3,0);
		joint_angle[0] = result.operator()(4,0);
	}else if(select_leg == "s_right"){
		ROS_INFO("[IK]s_right_ik_solver");
		TRAC_IK::TRAC_IK s_right_IK_solver("Right_link_foot", "waist", urdf_param, timeout, eps);
		valid = s_right_IK_solver.getKDLChain(chain);
		if (!valid) {
			ROS_ERROR("[IK]There was no valid KDL chain found");//有効なKDLチェーンが見つかりませんでした
			return;
		}
		valid = s_right_IK_solver.getKDLLimits(ll,ul);
		if (!valid) {
			ROS_ERROR("[IK]There were no valid KDL joint limits found");//有効なKDL関節限度が見つかりませんでした
			return;
		}
		//関節の限度が問題ないかのチェック
		assert(chain.getNrOfJoints() == ll.data.size());
		assert(chain.getNrOfJoints() == ul.data.size());
		// すべての関節限界の中間に名目上のチェーン構成を作成する
		KDL::JntArray s_right_nominal(chain.getNrOfJoints());
		for (uint j=0; j<s_right_nominal.data.size(); j++) {
			s_right_nominal(j) = (ll(j)+ul(j))/2.0;
		}
		rc = s_right_IK_solver.CartToJnt(s_right_nominal,end_effector_pose,result);
		if(rc>=0){
			ROS_INFO("[IK]%s IK is Success!",select_leg.c_str());
		}else{
			ROS_ERROR("[IK]%s IK is Error (rc = %d)",select_leg.c_str(),rc);
			ROS_ERROR("[IK]Calculation is not possible with that target value");
			return;
		}
		joint_angle[5] = result.operator()(0,0);
		joint_angle[6] = result.operator()(1,0);
		joint_angle[7] = result.operator()(2,0);
		joint_angle[8] = result.operator()(3,0);
		joint_angle[9] = result.operator()(4,0);
	}
	return;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "pwr_v2_walking_pattern_generator_node");
	ros::NodeHandle nh;
	ros::NodeHandle pn("~");
	double CoM_HIGHT = 0.24;
	double eps = 1e-5;
	double timeout = 0.005;
	double K = 1.0;					// ゲイン
	std::string urdf_param;
	pn.getParam("CoM_Hight", CoM_HIGHT);
	pn.getParam("timeout", timeout);
	pn.getParam("urdf_param", urdf_param);
	pn.getParam("gain", K);

	ros::Subscriber real_CP_position_sub = nh.subscribe("pwr_v2_CoM_pose",10,Sub_real_CoM_Callback);
	
	ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states",100);
	// ros::Publisher leg_mode_pub = nh.advertise<std_msgs::Float64>("leg_mode",10);
	// ros::Publisher left_foot_position_pub = nh.advertise<geometry_msgs::PointStamped>("left_foot_point",10);
	// ros::Publisher right_foot_position_pub = nh.advertise<geometry_msgs::PointStamped>("right_foot_point",10);
	// ros::Publisher LIP_CoM_position_pub = nh.advertise<geometry_msgs::PointStamped>("LIP_CoM_point",10);
	
	// 基準ZMPの設定
	MatrixXd ZMP_ref(3,step);	// 基準ZMP
	ZMP_ref <<  0.0, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
				0.0,-0.040, 0.040,-0.040, 0.040,-0.040, 0.040,
				0.0, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000;

	double omega = sqrt(G/CoM_HIGHT);					// 角速度
	double Hz = 1.0 / (limit_time / (double)division);	// ループ周期
	ROS_INFO("[WPG] Hz = %lf",Hz);
	ROS_INFO("[WPG] OMEGA = %lf",omega);

	// 基準CPの設定
	ROS_INFO("[WPG] Set Reference Capture Point");
	MatrixXd CP_ref_ini(3,step);	// 基準CP開始点
	MatrixXd CP_ref_end(3,step);	// 基準CP終点
	double num_time = limit_time * (double)step-1;
	double wTi;
	int i = step - 1;
	CP_ref_end.col(i-1) = ZMP_ref.col(i);
	i--;
	while(ros::ok()){
		wTi = -1.0 * omega * num_time;
		CP_ref_ini.col(i) = ZMP_ref.col(i) + exp(wTi) * (CP_ref_end.col(i) - ZMP_ref.col(i));
		if(i == 0)break;
		CP_ref_end.col(i-1) = CP_ref_ini.col(i);
		num_time -= limit_time;
		i--;
	}
	// std::cout << "wTi = "<< wTi << std::endl;
	// std::cout << "omega = "<< omega << std::endl;
	// std::cout << "num_time = "<< num_time << std::endl;
	// std::cout << "CP_ref_ini = \n"<< CP_ref_ini << std::endl;
	// std::cout << "CP_ref_end = \n"<< CP_ref_end << std::endl;

	// 基準CP軌道の設計
	ROS_INFO("[WPG] Set Reference Capture Point Load");
	double base_time = ros::Time::now().toSec();
	double time = 0.0;
	double wti;
	int data_count = (step-1) * division;
	MatrixXd CP_ref_ti(3,data_count);	// 基準CP軌道
	ROS_INFO("[WPG] data_count = %d",data_count);
	int s = 0;
	int j;
	ros::Rate loop_rate(Hz);
	loop_rate.sleep();
	for(i=0 ; i<(step-1) ; i++){
		for (j=0 ; j<division ; j++){
			wti = omega * time;
			CP_ref_ti.col(s) = ZMP_ref.col(i) + (exp(wti) * (CP_ref_ini.col(i) - ZMP_ref.col(i)));
			// ROS_INFO("[WPG] %d-%d(%d)",i,j,s);
			// std::cout << "wti = "<< wti << std::endl;
			// std::cout << "omega = "<< omega << std::endl;
			// std::cout << "time = "<< time << std::endl;
			// std::cout << "CP_ref_ini = \n"<< CP_ref_ini.col(i) << std::endl;
			std::cout << "CP_ref_ti = \n"<< CP_ref_ti.col(s) << std::endl;
			s++;
			time = ros::Time::now().toSec() - base_time;
			loop_rate.sleep();
		}
	}

	// 基準ZMP間をつなぐ遊脚の足先軌道を生成
	MatrixXd Free_Leg_position = MatrixXd::Zero(3,data_count-division);
	Free_Leg_position.col(0) = Left_foot_init_position; //左足先の初期位置
	// 最初の一歩目
	Vector3d start_position = Free_Leg_position.col(0);
	Vector3d end_position = ZMP_ref.col(2);
	// std::cout << "start_position(0)\n" << start_position <<std::endl;
	// std::cout << "end_position(2)\n" << end_position <<std::endl;
	double dx = (end_position(0) - start_position(0)) / (double)division;
	double dy = (end_position(1) - start_position(1)) / (double)division;
	double dz = 0.02 / ((double)division/2.0);
	// ROS_INFO("dx= %lf : dy= %lf : dz= %lf",dx,dy,dz);
	j = 1;
	for(i=1 ; i<division ; i++){
		Free_Leg_position(0,j) = Free_Leg_position(0,j-1) + dx;
		Free_Leg_position(1,j) = Free_Leg_position(1,j-1) + dy;
		if( i<(division/2) ) Free_Leg_position(2,j) = Free_Leg_position(2,j-1) + dz;
		else if( i>=(division/2) ) Free_Leg_position(2,j) = Free_Leg_position(2,j-1) - dz;
		// std::cout << "Free leg Position ("<< j << ") = \n"<< Free_Leg_position.col(j) <<std::endl;
		j++;
	}
	ROS_INFO("[WPG] Set Free Leg Load (1)");

	// 2歩目以降
	for (s=3 ; s<step ; s++){
		start_position = ZMP_ref.col(s-2);
		end_position = ZMP_ref.col(s);
		// std::cout << "start_position : " << s-2 << " \n" << start_position <<std::endl;
		// std::cout << "end_position : " << s << " \n" << end_position <<std::endl;
		dx = (end_position(0) - start_position(0)) / (double)division;
		dy = (end_position(1) - start_position(1)) / (double)division;
		// dz = 0.03 / ((double)division/2.0);
		// ROS_INFO("dx= %lf : dy= %lf : dz= %lf",dx,dy,dz);
		Free_Leg_position.col(j) = start_position;
		j++;
		for(i=1 ; i<division ; i++){
			Free_Leg_position(0,j) = Free_Leg_position(0,j-1) + dx;
			Free_Leg_position(1,j) = Free_Leg_position(1,j-1) + dy;
			if( i<(division/2) ) Free_Leg_position(2,j) = Free_Leg_position(2,j-1) + dz;
			else if( i>=(division/2) ) Free_Leg_position(2,j) = Free_Leg_position(2,j-1) - dz;
			// std::cout << "Free leg Position ("<< s << "-"<< i <<"-"<< j << ") =\n"<< Free_Leg_position.col(j) <<std::endl;
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
	double dt = limit_time/(double)division;
    MatrixXd Xx(2, 1);
    Xx(0, 0) = 0;
    Xx(1, 0) = 0;
	MatrixXd Xy(2, 1);
    Xy(0, 0) = 0;
    Xy(1, 0) = 0;
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


	s = 0;
	i = 1;
	j = 0;
	Hz = 20;
	ros::Rate loop_rate_2(Hz);
	ros::Time TIME;
	double _dt;
	double st = ros::Time::now().toSec();
	double RC=100.0;
	double Ts=20.0;
	Vector3d sensor(0.0, 0.0, 0.0650);
	Vector3d sensor_position;
	MatrixXd robot_CoM_pos = MatrixXd::Zero(3,3);
	MatrixXd robot_CoM_spd = MatrixXd::Zero(3,3);
	MatrixXd robot_CP = MatrixXd::Zero(3,1);
	ROS_INFO("[debug] set Matrix");

	int mode = 1;	// LIPモデルだけ：１　IMUを使用：２
	pn.getParam("mode", mode);

	std::ofstream fout_1("/home/shizuki/SynologyDrive/大学/修士論文/record/check_only_LIP.csv");
	fout_1 << "time,seq,CPref(ti)_x,CPref(ti)_y,ZMP_des_x,ZMP_des_y,CoM_position_x,CoM_positon_y,CP_x,CP_y" << std::endl;
	std::ofstream fout_2("/home/shizuki/SynologyDrive/大学/修士論文/record/check_use_IMU.csv");
	fout_2 << "time,seq,CPref(ti)_x,CPref(ti)_y,ZMP_des_x,ZMP_des_y,CoM_position_x,CoM_positon_y,CP_x,CP_y" << std::endl;
	ROS_INFO("[debug] set csv file");
	std::ofstream fout_3("/home/shizuki/SynologyDrive/大学/修士論文/record/check_LIP_and_IMU.csv");
	fout_3 << "time,seq,CPref(ti)_x,CPref(ti)_y,ZMP_des_x,ZMP_des_y,CoM_position_x,CoM_positon_y,(LIP)CP_x,(LIP)CP_y,(IMU)CP_x,(IMU)CP_y" << std::endl;
	ROS_INFO("[debug] set csv file");

	while(ros::ok()){
		TIME = ros::Time::now();
		// Ts = 1 /( st - TIME.toSec() );

		u(0,0) = ZMP_des(0,i-1);
		Xx = RungeKutta(dX, Xx, u, tt, dt, A, B, C, D);
        Yx = C*Xx;
		LIP_CoM_pos(0,i-1) = Yx(0);
		LIP_CoM_spd(0,1-1) = Yx(1);
		LIP_CP(0,i-1) = Yx(0) + (Yx(1)/omega);

		u(0,0) = ZMP_des(1,i-1);
		Xy = RungeKutta(dX, Xy, u, tt, dt, A, B, C, D);
        Yy = C*Xy;
		LIP_CoM_pos(1,i-1) = Yy(0);
		LIP_CoM_spd(1,i-1) = Yy(1);
		LIP_CP(1,i-1) = Yy(0) + (Yy(1)/omega);

		LIP_CoM_pos(2,i-1) = CoM_HIGHT;

		if(mode == 1){ // LIPモデルのみでZMPを修正
			if(i<=division){
				ROS_INFO("[IK]Sup:Right / Free:Left");
				IK_solver(nh, "s_right", LIP_CoM_pos.col(i-1)-Right_foot_init_position, urdf_param, timeout, eps, mode);
				IK_solver(nh, "f_left", Left_foot_init_position-LIP_CoM_pos.col(i-1), urdf_param, timeout, eps, mode);
				right_foot_msg = setPoint(Right_foot_init_position ,i-1,TIME,"Right_Foot");
				left_foot_msg = setPoint(Left_foot_init_position,i-1,TIME, "Left_Foot");
			}
			// Sup:Right / Free:Left
			else if(leg_mode_msg.data==1.0 && i>division){
				ROS_INFO("[IK]Sup:Right / Free:Left");
				IK_solver(nh, "s_right", LIP_CoM_pos.col(i-1)-ZMP_des.col(i-1), urdf_param, timeout, eps, mode);
				IK_solver(nh, "f_left", Free_Leg_position.col(j)-LIP_CoM_pos.col(i-1), urdf_param, timeout, eps, mode);
				right_foot_msg = setPoint(ZMP_des.col(i-1),i-1,TIME,"Right_Foot");
				left_foot_msg = setPoint(Free_Leg_position.col(j),i-1,TIME,"Left_Foot");
				// j += 5;
				j++;
			}
			// Sup:Left / Free:Right
			else if(leg_mode_msg.data==-1.0 && i>division){
				ROS_INFO("[IK]Sup:Left / Free:Right");
				IK_solver(nh, "s_left", LIP_CoM_pos.col(i-1)-ZMP_des.col(i-1), urdf_param, timeout, eps, mode);
				IK_solver(nh, "f_right", Free_Leg_position.col(j)-LIP_CoM_pos.col(i-1), urdf_param, timeout, eps, mode);
				right_foot_msg = setPoint(Free_Leg_position.col(j),i-1,TIME,"Right_Foot");
				left_foot_msg = setPoint(ZMP_des.col(i-1),i-1,TIME,"Left_Foot");
				// j += 5;
				j++;
			}
			waist_pose_publisher(LIP_CoM_pos.col(i-1), TIME, mode);
			waist_msg = setPoint(LIP_CoM_pos.col(i-1),i-1,TIME,"Waist");
			
			fout_1 << TIME.toSec() <<","<< i-1 <<","<<
			CP_ref_ti(0,i-1) <<","<< CP_ref_ti(1,i-1) <<","<<
			ZMP_des(0,i-1) <<","<< ZMP_des(1,i-1) <<","<< 
			LIP_CoM_pos(0,i-1) <<","<< LIP_CoM_pos(1,i-1) <<","<<
			LIP_CP(0,i-1) <<","<< LIP_CP(1,i-1) <<","<<
			std::endl;
		}
		else if(mode == 2){ // IMUの出力を使ってZMPを更新
			// センサの座標を求める
			ROS_INFO("[debug] sensor position");
			sensor_position = LIP_CoM_pos.col(i-1) + sensor;
			// 実機の重心座標をもとめる
			ROS_INFO("[debug] CoM position");
			robot_CoM_pos.col(1) = sensor_position + sub_CoM_vector;
			// 実機の重心位置の差から速度を求める
			ROS_INFO("[debug] CoM speed");
			_dt = st - TIME.toSec();
			robot_CoM_spd.col(1) = (robot_CoM_pos.col(1) - robot_CoM_pos.col(0)) / _dt;
			robot_CoM_spd.col(2) = LP_Filter(RC,Ts, robot_CoM_spd.col(1), robot_CoM_spd.col(0));
			//実機の Capture Point を求める
			ROS_INFO("[debug] capture point");
			robot_CP = robot_CoM_pos.col(1) + (robot_CoM_spd.col(2) / omega);
			robot_CP(2) = 0.0;
			// 次回の計算用に値を保持する
			robot_CoM_pos.col(0) = robot_CoM_pos.col(1);
			robot_CoM_spd.col(0) = robot_CoM_spd.col(2);

			if(i<=division){
				ROS_INFO("[IK]Sup:Right / Free:Left");
				IK_solver(nh, "s_right", robot_CoM_pos.col(1)-Right_foot_init_position, urdf_param, timeout, eps, mode);
				IK_solver(nh, "f_left", Left_foot_init_position-robot_CoM_pos.col(1), urdf_param, timeout, eps, mode);
				right_foot_msg = setPoint(Right_foot_init_position ,i-1,TIME,"Right_Foot");
				left_foot_msg = setPoint(Left_foot_init_position,i-1,TIME, "Left_Foot");
			}
			// Sup:Right / Free:Left
			else if(leg_mode_msg.data==1.0 && i>division){
				ROS_INFO("[IK]Sup:Right / Free:Left");
				IK_solver(nh, "s_right", robot_CoM_pos.col(1)-ZMP_des.col(i-1), urdf_param, timeout, eps, mode);
				IK_solver(nh, "f_left", Free_Leg_position.col(j)-robot_CoM_pos.col(1), urdf_param, timeout, eps, mode);
				right_foot_msg = setPoint(ZMP_des.col(i-1),i-1,TIME,"Right_Foot");
				left_foot_msg = setPoint(Free_Leg_position.col(j),i-1,TIME,"Left_Foot");
				// j += 5;
				j++;
			}
			// Sup:Left / Free:Right
			else if(leg_mode_msg.data==-1.0 && i>division){
				ROS_INFO("[IK]Sup:Left / Free:Right");
				IK_solver(nh, "s_left", robot_CoM_pos.col(1)-ZMP_des.col(i-1), urdf_param, timeout, eps, mode);
				IK_solver(nh, "f_right", Free_Leg_position.col(j)-robot_CoM_pos.col(1), urdf_param, timeout, eps, mode);
				right_foot_msg = setPoint(Free_Leg_position.col(j),i-1,TIME,"Right_Foot");
				left_foot_msg = setPoint(ZMP_des.col(i-1),i-1,TIME,"Left_Foot");
				// j += 5;
				j++;
			}
			waist_pose_publisher(robot_CoM_pos.col(1), TIME, mode);
			waist_msg = setPoint(robot_CoM_pos.col(1),i-1,TIME,"Waist");

			fout_2 << TIME.toSec() <<","<< i-1 <<","<<
			CP_ref_ti(0,i-1) <<","<< CP_ref_ti(1,i-1) <<","<<
			ZMP_des(0,i-1) <<","<< ZMP_des(1,i-1) <<","<< 
			robot_CoM_pos(0,1) <<","<< robot_CoM_pos(1,1) <<","<<
			robot_CP(0) <<","<< robot_CP(1) <<","<<
			std::endl;
		}
		else if(mode == 3){ // IMUで求めたCPの検証
			// センサの座標を求める
			ROS_INFO("[debug] sensor position");
			sensor_position = LIP_CoM_pos.col(i-1) + sensor;
			// 実機の重心座標をもとめる
			ROS_INFO("[debug] CoM position");
			robot_CoM_pos.col(1) = sensor_position + sub_CoM_vector;
			// 実機の重心位置の差から速度を求める
			ROS_INFO("[debug] CoM speed");
			_dt = st - TIME.toSec();
			robot_CoM_spd.col(1) = (robot_CoM_pos.col(1) - robot_CoM_pos.col(0)) / _dt;
			robot_CoM_spd.col(2) = LP_Filter(RC,Ts, robot_CoM_spd.col(1), robot_CoM_spd.col(0));
			//実機の Capture Point を求める
			ROS_INFO("[debug] capture point");
			robot_CP = robot_CoM_pos.col(1) + (robot_CoM_spd.col(1) / omega);
			robot_CP(2) = 0.0;
			// 次回の計算用に値を保持する
			robot_CoM_pos.col(0) = robot_CoM_pos.col(1);
			robot_CoM_spd.col(0) = robot_CoM_spd.col(2);

			if(i<=division){
				ROS_INFO("[IK]Sup:Right / Free:Left");
				IK_solver(nh, "s_right", LIP_CoM_pos.col(i-1)-Right_foot_init_position, urdf_param, timeout, eps, mode);
				IK_solver(nh, "f_left", Left_foot_init_position-LIP_CoM_pos.col(i-1), urdf_param, timeout, eps, mode);
				right_foot_msg = setPoint(Right_foot_init_position ,i-1,TIME,"Right_Foot");
				left_foot_msg = setPoint(Left_foot_init_position,i-1,TIME, "Left_Foot");
			}
			// Sup:Right / Free:Left
			else if(leg_mode_msg.data==1.0 && i>division){
				ROS_INFO("[IK]Sup:Right / Free:Left");
				IK_solver(nh, "s_right", LIP_CoM_pos.col(i-1)-ZMP_des.col(i-1), urdf_param, timeout, eps, mode);
				IK_solver(nh, "f_left", Free_Leg_position.col(j)-LIP_CoM_pos.col(i-1), urdf_param, timeout, eps, mode);
				right_foot_msg = setPoint(ZMP_des.col(i-1),i-1,TIME,"Right_Foot");
				left_foot_msg = setPoint(Free_Leg_position.col(j),i-1,TIME,"Left_Foot");
				// j += 5;
				j++;
			}
			// Sup:Left / Free:Right
			else if(leg_mode_msg.data==-1.0 && i>division){
				ROS_INFO("[IK]Sup:Left / Free:Right");
				IK_solver(nh, "s_left", LIP_CoM_pos.col(i-1)-ZMP_des.col(i-1), urdf_param, timeout, eps, mode);
				IK_solver(nh, "f_right", Free_Leg_position.col(j)-LIP_CoM_pos.col(i-1), urdf_param, timeout, eps, mode);
				right_foot_msg = setPoint(Free_Leg_position.col(j),i-1,TIME,"Right_Foot");
				left_foot_msg = setPoint(ZMP_des.col(i-1),i-1,TIME,"Left_Foot");
				// j += 5;
				j++;
			}
			waist_pose_publisher(LIP_CoM_pos.col(i-1), TIME, mode);
			waist_msg = setPoint(LIP_CoM_pos.col(i-1),i-1,TIME,"Waist");
			
			fout_3 << TIME.toSec() <<","<< i-1 <<","<<
			CP_ref_ti(0,i-1) <<","<< CP_ref_ti(1,i-1) <<","<<
			ZMP_des(0,i-1) <<","<< ZMP_des(1,i-1) <<","<< 
			LIP_CoM_pos(0,i-1) <<","<< LIP_CoM_pos(1,i-1) <<","<<
			LIP_CP(0,i-1) <<","<< LIP_CP(1,i-1) <<","<<
			robot_CP(0) <<","<< robot_CP(1) <<","<<
			std::endl;
		}
		
		joint_states_pub.publish(joint_state_publisher(joint_angle, TIME));
		// LIP_CoM_position_pub.publish(waist_msg);
		// right_foot_position_pub.publish(right_foot_msg);
		// left_foot_position_pub.publish(left_foot_msg);
		// leg_mode_pub.publish(leg_mode_msg);
		ROS_INFO("[WPG] Publish Point (i=%d, j=%d, leg_mode=%1.1f)",i,j,leg_mode_msg.data);
		// std::cout << waist_msg << std::endl;
		// std::cout << right_foot_msg << std::endl;
		// std::cout << left_foot_msg << std::endl;
		// std::cout << leg_mode_msg << std::endl;
		
		// if( (i+4)%division==0 ){
		// 	s++;
		// 	if((i+4)>=(2*division)){
		// 		leg_mode_msg.data = leg_mode_msg.data * -1.0;
		// 	}
		// }
		// if( (i+4)>=data_count )break;
		// if(mode==1||mode==3){
		// 	// CPの誤差を修正する所望のZMP(LIP model 使用)
		// 	ZMP_des.col(i+4) = ZMP_ref.col(s) + ( ( 1.0 + ( K/omega ) ) * ( LIP_CP.col(i-1) - CP_ref_ti.col(i-1) ) );
		// }
		// else if(mode==2){
		// 	// CPの誤差を修正する所望のZMP(bno055使用)
		// 	ZMP_des.col(i+4) = ZMP_ref.col(s) + ( (1.0 + (K/omega)) * ( robot_CP - CP_ref_ti.col(i-1) ) );
		// }
		// i += 5;
		if( i%division==0 ){
			s++;
			if(i>=(2*division)){
				leg_mode_msg.data = leg_mode_msg.data * -1.0;
			}
		}
		if( i>=data_count )break;
		if(mode==1||mode==3){
			// CPの誤差を修正する所望のZMP(LIP model 使用)
			ZMP_des.col(i) = ZMP_ref.col(s) + ( ( 1.0 + ( K/omega ) ) * ( LIP_CP.col(i-1) - CP_ref_ti.col(i-1) ) );
		}
		else if(mode==2){
			// CPの誤差を修正する所望のZMP(bno055使用)
			ZMP_des.col(i) = ZMP_ref.col(s) + ( (1.0 + (K/omega)) * ( robot_CP - CP_ref_ti.col(i-1) ) );
		}
		st = TIME.toSec();
		loop_rate_2.sleep();
		ros::spinOnce();
		i++;
	}
	ROS_INFO("[WPG] finish ");
	return 0;
}