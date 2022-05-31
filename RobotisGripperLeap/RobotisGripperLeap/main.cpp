#include <iostream>
#include <fstream>
#include <string.h>
#include "Leap.h"
#include "windows.h"
#include <stdio.h>
#include "serialcomm.h"
#include <conio.h>
#include "ARMSDK.h"
//#include "NIDAQmx.h"
#include "Pro_Arm_Comm_Win.h"

using namespace Leap;
using namespace std;
using namespace armsdk;

#define DEFAULT_CTRL_TIME_PERIOD 8

#ifdef _DEBUG
#pragma comment(lib, "ARMSDKd.lib")
#endif

#ifdef NDEBUG
#pragma comment(lib, "ARMSDK.lib") 
#endif

vecd gvdAngleGapCalcandDynamixelRad;

// 매니페스트 도구 > 입력 및 출력 > 매니페스트 포함 > 아니요
// 솔루션탐색기 > 속성 > 구성속성 > 일반 > 플랫폼 도구집합 > VS2010
// 속성 - 구성 속성 - 빌드 이벤트 - 빌드 후 이벤트 - 명령줄 > xcopy /yr "$(LEAP_SDK)\lib\x86\Leap.dll" "$(TargetDir)"

int main(int argc, char** argv) {
	RobotInfo RobotisArm;
	//               LinkLength  LinkTwist  JointOffset  JointAngle
	RobotisArm.AddJoint(0.0, -ML_PI_2, 0.0, 0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 1);
	RobotisArm.AddJoint(265.7, 0.0, 0.0, 0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 2);
	RobotisArm.AddJoint(30.0, -ML_PI_2, 0.0, 0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 3);
	RobotisArm.AddJoint(0.0, ML_PI_2, 258.0, 0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 4);
	RobotisArm.AddJoint(0.0, -ML_PI_2, 0.0, 0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 5);
	RobotisArm.AddJoint(0.0, 0.0, 0.0, 0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 6);

	RobotInfo GripperMX;
	GripperMX.AddJoint(0.0, 0.0, 0.0, 0.0, 1, 0, 2402, 1951, 1, 0, 7);//클수록 폄
	GripperMX.AddJoint(0.0, 0.0, 0.0, 0.0, 1, 0, 2125, 1782, 1, 0, 8);//클수록 쥠!!
	GripperMX.AddJoint(0.0, 0.0, 0.0, 0.0, 1, 0, 2390, 1940, 1, 0, 9);//클수록 폄

	//gvdAngleGapCalcandDynamixelRad for Manipulator
	gvdAngleGapCalcandDynamixelRad.resize(6);
	gvdAngleGapCalcandDynamixelRad << 0.0, ML_PI_2 - 6.4831*ML_PI / 180.0, ML_PI_4 + 6.4831*ML_PI / 180.0, 0.0, 0.0, 0.0;

	Kinematics ArmKinematics(&RobotisArm);
	Kinematics GripperKinematics(&GripperMX);
	ArmKinematics.SetMaximumNumberOfIterationsForIK(50);
	ArmKinematics.SetConvergenceCondition(0.001, 5.0);

	vecd angle_rad;
	vecd angle_rad_gr;
	angle_rad.resize(RobotisArm.GetRobotInfo()->size());
	angle_rad_gr.resize(GripperMX.GetRobotInfo()->size());

	veci angle_unit;
	veci angle_unit_gr;
	angle_unit.resize(RobotisArm.GetRobotInfo()->size());
	angle_unit_gr.resize(GripperMX.GetRobotInfo()->size());

	vecd InitialPose;
	InitialPose.resize(RobotisArm.GetRobotInfo()->size());
	InitialPose << 0, -ML_PI + 6.4831*ML_PI / 180.0, -ML_PI/3.0f - 6.4831*ML_PI / 180.0, 0.0, ML_PI/3.0f, 0;
	//InitialPose << 0, -ML_PI + 6.4831*ML_PI / 180.0, -ML_PI_4 - 6.4831*ML_PI / 180.0, 0.0, ML_PI_4, 0;
	//InitialPose << 0, -ML_PI_2 + 6.4831*ML_PI / 180.0, -ML_PI_2 - 6.4831*ML_PI / 180.0, 0.0, 0, 0;
	vecd TwistPose;
	TwistPose.resize(RobotisArm.GetRobotInfo()->size());
	TwistPose << 0, -ML_PI + 6.4831*ML_PI / 180.0, -ML_PI / 3.0f - 6.4831*ML_PI / 180.0, ML_PI/8.0f, ML_PI /4.0f, -ML_PI / 6.0f;

	vecd G1, G2;
	G1.resize(GripperMX.GetRobotInfo()->size());
	G2.resize(GripperMX.GetRobotInfo()->size());
	G1 << 0.2, 0.8, 0.2; // 쥔 상태
	G2 << 0.8, 0.1, 0.8; // 편 상태

	Pro_Arm_Comm_Win ArmComm;
	int Port = 21, Baud = 3;

	ArmComm.DXL_Set_Init_Param(Port, Baud);
	ArmComm.Arm_ID_Setup(RobotisArm.GetArmIDList());
	ArmComm.Gripper_ID_Setup(GripperMX.GetArmIDList());

	if (ArmComm.DXL_Open() == 0)
		std::cout << "Failed to open USB2Dynamixel" << endl;
	else
		std::cout << "Succeed to open USB2Dynamixel" << endl;

	ArmComm.Arm_Torque_On();
	ArmComm.Gripper_Torque_On();

	if (ArmComm.Arm_Get_JointPosition(&angle_unit) != COMM_RXSUCCESS)
		return -1;
	if (ArmComm.Gripper_Get_Joint_Value(&angle_unit_gr) != COMM_RXSUCCESS)
		return -1;

	for (int JointIdx = 0; JointIdx < RobotisArm.GetRobotInfo()->size(); JointIdx++)
	{
		if (abs(angle_unit[JointIdx]) > RobotisArm.GetJointInfo(JointIdx).GetMaxAngleInValue())
		{
			std::cout << "Joint Angle Read Fail" << endl;
			_getch();
			return -1;
		}
	}

	for (int JointIdx = 0; JointIdx < GripperMX.GetRobotInfo()->size(); JointIdx++)
	{
		if (abs(angle_unit_gr[JointIdx]) > GripperMX.GetJointInfo(JointIdx).GetMaxAngleInValue())
		{
			std::cout << "Joint Angle Read Fail in Gripper" << endl;
			_getch();
			return -1;
		}
	}

	ArmComm.Arm_Set_Position_PID_Gain(64, 0, 0);
	ArmComm.Gripper_Set_Position_PID_Gain(10, 0, 0);

	ArmComm.Arm_Set_JointVelocity(0);//Maximum Velocity
	ArmComm.Arm_Set_JointAcceleration(0);//Maximum Acceleration

	angle_rad = RobotisArm.Value2Rad(angle_unit);
	angle_rad_gr = GripperMX.Value2Rad(angle_unit_gr);

	TrajectoryGenerator ArmTrajectory(&ArmKinematics);
	TrajectoryGenerator GripperTrajectory(&GripperKinematics);

	// Set Initial Position
	ArmTrajectory.Set_P2P(angle_rad - gvdAngleGapCalcandDynamixelRad, InitialPose, 3.0, 1.0);
	GripperTrajectory.Set_P2P(angle_rad_gr, G2, 3.0, 1.0);

	MotionPlay MotionPlayer(&ArmKinematics, &ArmTrajectory);
	MotionPlay MotionPlayer_G(&GripperKinematics, &GripperTrajectory);
	MotionPlayer.Set_Time_Period(DEFAULT_CTRL_TIME_PERIOD);
	MotionPlayer_G.Set_Time_Period(DEFAULT_CTRL_TIME_PERIOD);
	MotionTimer timer;

	int ErrorStatus, Result, Result_G;
	std::cout << "Press any key to move first pose" << endl;
	_getch();
	std::cout << "move to first pose" << endl;
	
	//// Write File for Torque Data
	ofstream fileout("Torque_0820_1.txt");
	veci arm_Torque, arm_Torque_Direction;
	veci gripper_Torque, gripper_Torque_Direction;
	arm_Torque.resize(RobotisArm.GetRobotInfo()->size());
	arm_Torque_Direction.resize(RobotisArm.GetRobotInfo()->size());
	gripper_Torque.resize(GripperMX.GetRobotInfo()->size());
	gripper_Torque_Direction.resize(GripperMX.GetRobotInfo()->size());

	// Gripper 와 Arm 같이 사용하는 것 성공
	while (MotionPlayer.Get_CurrentTime() < ArmTrajectory.GetMotionTotalTime())
	{
		timer.Start();
		angle_rad = MotionPlayer.NextStep(&ErrorStatus);
		angle_unit = RobotisArm.Rad2Value(angle_rad + gvdAngleGapCalcandDynamixelRad);
		angle_rad_gr = MotionPlayer_G.NextStep(&ErrorStatus);
		angle_unit_gr = GripperMX.Rad2Value(angle_rad_gr);

		Result = ArmComm.Arm_Set_JointPosition(angle_unit);
		Result_G = ArmComm.Gripper_Set_Joint_Value(angle_unit_gr);
		
		// Torque 측정 (시작)
		ArmComm.Arm_Get_JointCurrent(&arm_Torque);
		ArmComm.Gripper_Get_Load_Value(&gripper_Torque);
		
		// fileout << "TORQUE @ FRAME  " << MotionPlayer.Get_CurrentTime() << endl << arm_Torque << endl << gripper_Torque << endl;
		// Torque 측정 (끝)
		

		if (Result != COMM_RXSUCCESS || Result_G != COMM_RXSUCCESS)
		{
			std::cout << "Connection Error" << endl;
			return -1;
		}
		timer.Stop();
		timer.Wait(DEFAULT_CTRL_TIME_PERIOD - timer.GetElapsedTime());
	}

	
	////Clear MotionProfile and Set New Motion Profile
	ArmTrajectory.ClearMF();
	GripperTrajectory.ClearMF();

	// Set Grasping Position
	if (ArmComm.Arm_Get_JointPosition(&angle_unit) != COMM_RXSUCCESS)
		return -1;
	if (ArmComm.Gripper_Get_Joint_Value(&angle_unit_gr) != COMM_RXSUCCESS)
		return -1;
	angle_rad = RobotisArm.Value2Rad(angle_unit);
	angle_rad_gr = GripperMX.Value2Rad(angle_unit_gr);

	ArmTrajectory.Set_P2P(angle_rad - gvdAngleGapCalcandDynamixelRad, InitialPose, 3.0, 1.0);
	GripperTrajectory.Set_P2P(angle_rad_gr, G1, 3.0, 1.0);

	MotionPlayer.All_Info_Reload();
	MotionPlayer.Initialize();
	MotionPlayer_G.All_Info_Reload();
	MotionPlayer_G.Initialize();
	MotionPlayer.Set_Time_Period(DEFAULT_CTRL_TIME_PERIOD);
	MotionPlayer_G.Set_Time_Period(DEFAULT_CTRL_TIME_PERIOD);

	std::cout << "Press any key to Grasp" << endl;
	_getch();
	std::cout << "Grasping..." << endl;

	// Gripper 와 Arm 같이 사용하는 것 성공
	while (MotionPlayer.Get_CurrentTime() < ArmTrajectory.GetMotionTotalTime())
	{
		// Torque 측정 (시작)
		ArmComm.Arm_Get_JointCurrent(&arm_Torque);
		ArmComm.Gripper_Get_Load_Value(&gripper_Torque);
		if (gripper_Torque[0]> 1024)	{
			gripper_Torque_Direction[0] = -1; // Close;
			gripper_Torque[0] -= 1024;
		}
		else {
			gripper_Torque_Direction[0] = 1; // Open;
		}
		if (gripper_Torque[1]> 1024)	{
			gripper_Torque_Direction[1] = 1; // Open;
			gripper_Torque[1] -= 1024;
		}
		else {
			gripper_Torque_Direction[1] = -1; // Close;
		}
		if (gripper_Torque[2]> 1024)	{
			gripper_Torque_Direction[2] = -1; // Close;
			gripper_Torque[2] -= 1024;
		}
		else {
			gripper_Torque_Direction[2] = 1; // Open;
		}
		/*
		cout << "Current: ";
		for (int j = 0; j < RobotisArm.GetRobotInfo()->size(); j++){
		cout << arm_Torque[j] << "   ";
		}
		cout << "           ";
		for (int j = 0; j < GripperMX.GetRobotInfo()->size(); j++){
		cout << gripper_Torque[j] << "   ";
		}
		cout << endl;
		*/
		fileout << "TORQUE @ FRAME  " << MotionPlayer.Get_CurrentTime() << endl << arm_Torque << endl << gripper_Torque << endl;
		// Torque 측정 (끝)
		veci prv_angle_unit_gr;
		prv_angle_unit_gr.resize(GripperMX.GetRobotInfo()->size());
		prv_angle_unit_gr = angle_unit_gr;

		timer.Start();
		angle_rad = MotionPlayer.NextStep(&ErrorStatus);
		angle_unit = RobotisArm.Rad2Value(angle_rad + gvdAngleGapCalcandDynamixelRad);
		angle_rad_gr = MotionPlayer_G.NextStep(&ErrorStatus);
		angle_unit_gr = GripperMX.Rad2Value(angle_rad_gr);
		for (int i = 0; i < 3; i++){
			if (gripper_Torque[i]>200){
				angle_unit_gr[i] = prv_angle_unit_gr[i];
			}
		}

		Result = ArmComm.Arm_Set_JointPosition(angle_unit);
		Result_G = ArmComm.Gripper_Set_Joint_Value(angle_unit_gr);

		if (Result != COMM_RXSUCCESS || Result_G != COMM_RXSUCCESS)
		{
			std::cout << "Connection Error" << endl;
			return -1;
		}
		timer.Stop();
		timer.Wait(DEFAULT_CTRL_TIME_PERIOD - timer.GetElapsedTime());
	}

	////Clear MotionProfile and Set New Motion Profile
	ArmTrajectory.ClearMF();
	GripperTrajectory.ClearMF();

	// Torque file close	
	fileout.close();
	//Torque off 시키는 코드
	ArmComm.Arm_Torque_Off();
	ArmComm.Gripper_Torque_Off();
	return 1;

	////// Twist wrist
	if (ArmComm.Arm_Get_JointPosition(&angle_unit) != COMM_RXSUCCESS)
		return -1;
	if (ArmComm.Gripper_Get_Joint_Value(&angle_unit_gr) != COMM_RXSUCCESS)
		return -1;
	angle_rad = RobotisArm.Value2Rad(angle_unit);
	angle_rad_gr = GripperMX.Value2Rad(angle_unit_gr);

	ArmTrajectory.Set_P2P(angle_rad - gvdAngleGapCalcandDynamixelRad, TwistPose, 3.0, 1.0);
	GripperTrajectory.Set_P2P(angle_rad_gr, G1, 3.0, 1.0);

	MotionPlayer.All_Info_Reload();
	MotionPlayer.Initialize();
	MotionPlayer_G.All_Info_Reload();
	MotionPlayer_G.Initialize();
	MotionPlayer.Set_Time_Period(DEFAULT_CTRL_TIME_PERIOD);
	MotionPlayer_G.Set_Time_Period(DEFAULT_CTRL_TIME_PERIOD);

	std::cout << "Press any key to Grasp" << endl;
	_getch();
	std::cout << "Grasping..." << endl;

	// Gripper 와 Arm 같이 사용하는 것 성공
	while (MotionPlayer.Get_CurrentTime() < ArmTrajectory.GetMotionTotalTime())
	{
		// Torque 측정 (시작)
		ArmComm.Arm_Get_JointCurrent(&arm_Torque);
		ArmComm.Gripper_Get_Load_Value(&gripper_Torque);
		if (gripper_Torque[0]> 1024)	{
			gripper_Torque_Direction[0] = -1; // Close;
			gripper_Torque[0] -= 1024;
		}
		else {
			gripper_Torque_Direction[0] = 1; // Open;
		}
		if (gripper_Torque[1]> 1024)	{
			gripper_Torque_Direction[1] = 1; // Open;
			gripper_Torque[1] -= 1024;
		}
		else {
			gripper_Torque_Direction[1] = -1; // Close;
		}
		if (gripper_Torque[2]> 1024)	{
			gripper_Torque_Direction[2] = -1; // Close;
			gripper_Torque[2] -= 1024;
		}
		else {
			gripper_Torque_Direction[2] = 1; // Open;
		}
		/*
		cout << "Current: ";
		for (int j = 0; j < RobotisArm.GetRobotInfo()->size(); j++){
		cout << arm_Torque[j] << "   ";
		}
		cout << "           ";
		for (int j = 0; j < GripperMX.GetRobotInfo()->size(); j++){
		cout << gripper_Torque[j] << "   ";
		}
		cout << endl;
		*/
		fileout << "TORQUE @ FRAME  " << MotionPlayer.Get_CurrentTime() << endl << arm_Torque << endl << gripper_Torque << endl;
		// Torque 측정 (끝)
		veci prv_angle_unit_gr;
		prv_angle_unit_gr.resize(GripperMX.GetRobotInfo()->size());
		prv_angle_unit_gr = angle_unit_gr;

		timer.Start();
		angle_rad = MotionPlayer.NextStep(&ErrorStatus);
		angle_unit = RobotisArm.Rad2Value(angle_rad + gvdAngleGapCalcandDynamixelRad);
		angle_rad_gr = MotionPlayer_G.NextStep(&ErrorStatus);
		angle_unit_gr = GripperMX.Rad2Value(angle_rad_gr);
		for (int i = 0; i < 3; i++){
			if (gripper_Torque[i]>200){
				angle_unit_gr[i] = prv_angle_unit_gr[i];
			}
		}

		Result = ArmComm.Arm_Set_JointPosition(angle_unit);
		Result_G = ArmComm.Gripper_Set_Joint_Value(angle_unit_gr);

		if (Result != COMM_RXSUCCESS || Result_G != COMM_RXSUCCESS)
		{
			std::cout << "Connection Error" << endl;
			return -1;
		}
		timer.Stop();
		timer.Wait(DEFAULT_CTRL_TIME_PERIOD - timer.GetElapsedTime());
	}

	////Clear MotionProfile and Set New Motion Profile
	ArmTrajectory.ClearMF();
	GripperTrajectory.ClearMF();

	/*
	//// Test for force
	ofstream fileout("Torque.txt");
	veci arm_Torque;
	veci gripper_Torque;
	arm_Torque.resize(RobotisArm.GetRobotInfo()->size());
	gripper_Torque.resize(GripperMX.GetRobotInfo()->size());

	Pose3D DesiredPose = ArmKinematics.GetCurrentPose(); 
	cout << "Desired pose: " << DesiredPose.x << " ,  " << DesiredPose.y << " ,  " << DesiredPose.z << " ,  " << DesiredPose.roll << " ,  " << DesiredPose.pitch << " ,  " << DesiredPose.yaw << endl;

	float robot_delta = 0;
	int i = 0;
	while (1){
		ArmKinematics.ComputeIK(DesiredPose, &angle_rad, angle_rad, &ErrorStatus);
		ArmComm.Arm_Get_JointCurrent(&arm_Torque);
		ArmComm.Gripper_Get_Load_Value(&gripper_Torque);
		cout << "Current: ";
		for (int j = 0; j < RobotisArm.GetRobotInfo()->size(); j++){
			cout << arm_Torque[j] << "   ";
		}
		cout << "           ";
		for (int j = 0; j < GripperMX.GetRobotInfo()->size(); j++){
			cout << gripper_Torque[j] << "   ";
		}
		cout << endl;
		fileout << "TORQUE @ FRAME  " << i << arm_Torque << endl << gripper_Torque << endl;
		if (ErrorStatus == ARMSDK_NO_ERROR) {	
			ArmComm.Arm_Set_JointPosition(RobotisArm.Rad2Value(angle_rad + gvdAngleGapCalcandDynamixelRad));
		}
		else
		{
			std::cout << "No IK Solution" << endl;
			continue;
		}

		ArmKinematics.Forward(angle_rad, &DesiredPose);
		i++;
		if (i > 1000) break;
	}
	fileout.close();

	//Torque off 시키는 코드
	ArmComm.Arm_Torque_Off();
	ArmComm.Gripper_Torque_Off();
	return 1;
	*/
		

	
	// Leap motion 설정
	Controller controller;
	int64_t lastFrameID = 0;

	std::cout << "Press any key to initialize pose" << endl;
	_getch();
	std::cout << "Initialize start" << endl;
	
	//Initial position
	Vector prepos;
	Vector pos;
	float pitch = 0;
	float yaw = 0;
	float roll = 0;
	for (int i = 0; i < 10; i++){
		Leap::Frame frame = controller.frame();
		if (frame.id() != lastFrameID){
			Leap::HandList hands = frame.hands();
			Leap::Hand hand = hands.rightmost();
			pos = hand.palmPosition() + prepos;
			//std::cout << i << "  " << pos << endl;
			prepos = pos;
			pitch = hand.direction().pitch() + pitch;
			yaw = hand.direction().yaw() + yaw;
			roll = hand.palmNormal().roll() + roll;
			Sleep(100);
		}
		lastFrameID = frame.id();
	}
	vecd Ipos;
	Ipos.resize(3);
	Ipos << pos.x / 10, pos.y / 10, pos.z / 10;
	float I_pitch = pitch / 10; float I_yaw = yaw / 10; float I_roll = roll / 10;
	std::cout << "pos : " << Ipos << endl << "pitch : " << I_pitch << "yaw : " << I_yaw << "roll :" << I_roll << endl;
	std::cout << "Initialize done" << endl;

	Pose3D Robot_Ipos;
	Robot_Ipos = ArmKinematics.GetCurrentPose();
	//angle_rad = ArmKinematics.GetCurrentAngle();

	//Start Leap control
	std::cout << "Press any key to start control" << endl;
	_getch();
	std::cout << "Control started" << endl;

	veci load_gr;
	load_gr.resize(3);
	vecd looseG1;
	looseG1.resize(3);
	looseG1 = G1;
	veci flag_load;
	flag_load.resize(3);
	flag_load << 0, 0, 0;
	int loose_on = 0;

	while (true){

		Leap::Frame frame = controller.frame();
		Leap::HandList hands = frame.hands();
		int handcount = frame.hands().count();
		Pose3D DesiredPose;
		Pose3D DesiredPose_gr;

		if (handcount == 1){ // hand가 하나인 경우
			angle_rad = ArmKinematics.GetCurrentAngle();//현재 팔 각도 읽어오기
			angle_rad_gr = GripperKinematics.GetCurrentAngle();//현재 손가락 각도 읽어오기

			ArmComm.Arm_Set_JointAcceleration(500000000);//가속도:4, -21 4748 3648 ~ 21 4748 3647 
			ArmComm.Arm_Set_JointVelocity(800);//속도 :500, 0~1023

			Leap::Hand hand = hands[0];
			Leap::Vector pos = hand.palmPosition();

			roll = hand.palmNormal().roll();
			pitch = hand.direction().pitch();
			yaw = hand.direction().yaw();

			float grabAngle = hand.grabAngle(); 

			int workspaceratio = 1.2;
			double hand_delta_x = pos[0] - Ipos[0];
			double hand_delta_y = pos[1] - Ipos[1];
			double hand_delta_z = pos[2] - Ipos[2];
			double robot_delta_x = hand_delta_y*workspaceratio;
			double robot_delta_y = -hand_delta_x*workspaceratio;
			double robot_delta_z = hand_delta_z*workspaceratio;
			float hand_delta_roll = roll - I_roll;
			float hand_delta_pitch = pitch - I_pitch;
			float hand_delta_yaw = yaw - I_yaw;

			hand_delta_roll = (hand_delta_roll > 2) ? hand_delta_roll - PI : hand_delta_roll;
			hand_delta_roll = (hand_delta_roll < -2) ? hand_delta_roll + PI : hand_delta_roll;

			hand_delta_pitch = (hand_delta_pitch > 2) ? hand_delta_pitch - PI : hand_delta_pitch;
			hand_delta_pitch = (hand_delta_pitch < -2) ? hand_delta_pitch + PI : hand_delta_pitch;

			hand_delta_yaw = (hand_delta_yaw > 2) ? hand_delta_yaw - PI : hand_delta_yaw;
			hand_delta_yaw = (hand_delta_yaw < -2) ? hand_delta_yaw + PI : hand_delta_yaw;

			////////////start : 각도를 직접 대입 할 경우 설정////////////
			float robot_delta_roll = (-hand_delta_roll)*0.5;
			float robot_delta_pitch = (hand_delta_pitch)*0.5;
			float robot_delta_yaw = (-hand_delta_yaw)*0.5;
			cout << robot_delta_roll << " " << robot_delta_pitch << " " << robot_delta_yaw << endl;
			////////////end :각도를 직접 대입 할 경우 설정////////////

			matd DesiredRotation = Algebra::GetOrientationMatrix(robot_delta_roll, robot_delta_pitch, robot_delta_yaw) * Algebra::GetOrientationMatrix(Robot_Ipos.roll, Robot_Ipos.pitch, Robot_Ipos.yaw);
			vecd DesiredRPY = Algebra::GetEulerRollPitchYaw(DesiredRotation);

			DesiredPose.roll = DesiredRPY(0);
			DesiredPose.pitch = DesiredRPY(1);
			DesiredPose.yaw = DesiredRPY(2);

			DesiredPose.x = Robot_Ipos.x + robot_delta_x;
			DesiredPose.y = Robot_Ipos.y + robot_delta_y;
			DesiredPose.z = Robot_Ipos.z + robot_delta_z;

			ArmKinematics.ComputeIK(DesiredPose, &angle_rad, angle_rad, &ErrorStatus);
			/*
			if (ErrorStatus == ARMSDK_NO_ERROR) {
				//cout << "Answer" << endl;
				//cout << angle_rad << endl << endl;

				//////////start : 각도 직접 대입 부분(1)//////////
				angle_rad[3] = yaw2 / 2;
				angle_rad[4] = (ML_PI_4 + pitch2 / 2);
				angle_rad[5] = -roll2;
				//////////end : 각도 직접 대입 부분(1)//////////

				ArmComm.Arm_Set_JointPosition(RobotisArm.Rad2Value(angle_rad + gvdAngleGapCalcandDynamixelRad));
			}
			else
			{
				std::cout << "No IK Solution" << endl;
				continue;
			}
			*/

			ArmKinematics.Forward(angle_rad, &DesiredPose);

			ArmComm.Arm_Set_JointPosition(RobotisArm.Rad2Value(angle_rad + gvdAngleGapCalcandDynamixelRad));

			//////start : gripper 움직임 설정////////
			if (grabAngle < ML_PI_2){ // 편 손
				ArmComm.Gripper_Get_Joint_Value(&angle_unit_gr);
				angle_rad_gr = GripperMX.Value2Rad(angle_unit_gr);

				ArmComm.Gripper_Set_Joint_Value(GripperMX.Rad2Value(G2));// G1:쥔,G2:편
				loose_on = 0;
			}
			
			else{ // 쥔 손
				ArmComm.Gripper_Get_Joint_Value(&angle_unit_gr);
				angle_rad_gr = GripperMX.Value2Rad(angle_unit_gr);
				if (loose_on==0)	ArmComm.Gripper_Set_Joint_Value(GripperMX.Rad2Value(G1));// G1:쥔,G2:편
				else ArmComm.Gripper_Set_Joint_Value(GripperMX.Rad2Value(looseG1));// G1:쥔,G2:편
			}
			
			ArmComm.Gripper_Get_Load_Value(&load_gr);
			for (int i = 0; i < 3; i++){
				if (load_gr[i] % 1024 > 1024 * 0.4) flag_load[i]++;
				if (flag_load[i] > 8) {
					looseG1[i] = (G1[i] - angle_rad_gr[i])*0.1 + angle_rad_gr[i];
					flag_load[i] = 0;
					loose_on = 0;
				}
			}
			//출력 테스트
			if (loose_on == 0) std::cout << "Load[  ]";
			else std::cout << "Load[on]";
			std::cout << load_gr[0] << "  " << load_gr[1] << "  " << load_gr[2] << endl;
			std::cout << "flag:  " << flag_load[0] << "  " << flag_load[1] << "  " << flag_load[2] << endl;
			std::cout << "looseG1:  " << looseG1[0] << "  " << looseG1[1] << "  " << looseG1[2] << endl;
			//////end : gripper 움직임 설정////////
		}
		//hand가 하나가 아닌 경우
		else
		{
			std::cout << "Homing...." << endl;
			angle_rad = RobotisArm.Value2Rad(angle_unit);
			angle_rad_gr = GripperMX.Value2Rad(angle_unit_gr);

			TrajectoryGenerator ArmTrajectory(&ArmKinematics);
			TrajectoryGenerator GripperTrajectory(&GripperKinematics);

			ArmTrajectory.ClearMF();
			GripperTrajectory.ClearMF();

			ArmTrajectory.Set_P2P(angle_rad - gvdAngleGapCalcandDynamixelRad, InitialPose, 2.0, 1.0);
			GripperTrajectory.Set_P2P(angle_rad_gr, G2, 2.0, 1.0);

			MotionPlay MotionPlayer(&ArmKinematics, &ArmTrajectory);
			MotionPlay MotionPlayer_G(&GripperKinematics, &GripperTrajectory);
			MotionPlayer.Set_Time_Period(DEFAULT_CTRL_TIME_PERIOD);
			MotionPlayer_G.Set_Time_Period(DEFAULT_CTRL_TIME_PERIOD);
			MotionTimer timer;

			int ErrorStatus, Result, Result_G;

			while (MotionPlayer.Get_CurrentTime() < ArmTrajectory.GetMotionTotalTime())
			{
				timer.Start();
				angle_rad = MotionPlayer.NextStep(&ErrorStatus);
				angle_unit = RobotisArm.Rad2Value(angle_rad + gvdAngleGapCalcandDynamixelRad);
				angle_rad_gr = MotionPlayer_G.NextStep(&ErrorStatus);
				angle_unit_gr = GripperMX.Rad2Value(angle_rad_gr);

				Result = ArmComm.Arm_Set_JointPosition(angle_unit);
				Result_G = ArmComm.Gripper_Set_Joint_Value(angle_unit_gr);

				if (Result != COMM_RXSUCCESS || Result_G != COMM_RXSUCCESS)
				{
					std::cout << "Connection Error" << endl;
					return -1;
				}
				
				timer.Stop();
				timer.Wait(DEFAULT_CTRL_TIME_PERIOD - timer.GetElapsedTime());
			}

			////Clear MotionProfile and Set New Motion Profile
			ArmTrajectory.ClearMF();
			GripperTrajectory.ClearMF();

			std::cout << "Press any key to initialize pose" << endl;
			_getch();
			std::cout << "Initialize start" << endl;

			Vector prepos;
			Vector pos;
			pitch = 0; yaw = 0; roll = 0;
			//Initial position
			for (int i = 0; i < 10; i++){
				Leap::Frame frame = controller.frame();
				if (frame.id() != lastFrameID){
					Leap::HandList hands = frame.hands();
					Leap::Hand hand = hands.rightmost();
					pos = hand.palmPosition() + prepos;
					//std::cout << i << "  " << pos << endl;
					prepos = pos;
					pitch = hand.direction().pitch() + pitch;
					yaw = hand.direction().yaw() + yaw;
					roll = hand.palmNormal().roll() + roll;
					Sleep(100);

				}
				lastFrameID = frame.id();
			}

			vecd Ipos;
			Ipos.resize(3);
			Ipos << pos.x / 10, pos.y / 10, pos.z / 10;
			I_pitch = pitch / 10; I_yaw = yaw / 10; I_roll = roll / 10;
			std::cout << "pos : " << Ipos << endl << "pitch : " << I_pitch << "yaw : " << I_yaw << "roll :" << I_roll << endl;
			std::cout << "Initialize done" << endl;

			Pose3D Robot_Ipos;
			Robot_Ipos = ArmKinematics.GetCurrentPose();
			//angle_rad = ArmKinematics.GetCurrentAngle();

			std::cout << "Press any key to start control, Press 'ESC' to stop." << endl;
			char c = _getch();
			if (c == 27){
				//Torque off 시키는 코드
				ArmComm.Arm_Torque_Off();
				ArmComm.Gripper_Torque_Off();
				return 1;
			}
			else
				continue;
			std::cout << "Control started" << endl;

		}
		
	}
}
