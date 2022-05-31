// SimpleP2P.exe
// 
// By referring to the link below, please change the USB2Dynamixel's Latency Timer to 1ms.
// http://support.robotis.com/en/software/dynamixel_sdk/usb2dynamixel/usb2dxl_windows.htm
//

#include <iostream>
#include <conio.h>
#include "ARMSDK.h"
#include "Pro_Arm_Comm_Win.h"

using namespace armsdk;
using namespace std;

#define DEFAULT_CTRL_TIME_PERIOD 8

#ifdef _DEBUG
#pragma comment(lib, "ARMSDKd.lib")
#endif

#ifdef NDEBUG
#pragma comment(lib, "ARMSDK.lib") 
#endif

vecd gvdAngleGapCalcandDynamixelRad;

int main(int argc, char** argv) {
{
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
	InitialPose << 0, -ML_PI + 6.4831*ML_PI / 180.0, -ML_PI / 3.0f - 6.4831*ML_PI / 180.0, 0.0, ML_PI / 3.0f, 0;
	//InitialPose << 0, -ML_PI + 6.4831*ML_PI / 180.0, -ML_PI_4 - 6.4831*ML_PI / 180.0, 0.0, ML_PI_4, 0;
	//InitialPose << 0, -ML_PI_2 + 6.4831*ML_PI / 180.0, -ML_PI_2 - 6.4831*ML_PI / 180.0, 0.0, 0, 0;
	vecd TwistPose;
	TwistPose.resize(RobotisArm.GetRobotInfo()->size());
	TwistPose << 0, -ML_PI + 6.4831*ML_PI / 180.0, -ML_PI / 3.0f - 6.4831*ML_PI / 180.0, ML_PI / 8.0f, ML_PI / 4.0f, -ML_PI / 6.0f;

	vecd G1, G2;
	G1.resize(GripperMX.GetRobotInfo()->size());
	G2.resize(GripperMX.GetRobotInfo()->size());
	G1 << 0.2, 0.8, 0.2; // 쥔 상태
	G2 << 0.8, 0.1, 0.8; // 편 상태

	Pro_Arm_Comm_Win ArmComm;
	int Port, Baud;
	cout << "Input COM port number : ";
	cin >> Port;
	cout << "Input baud number : ";
	cin >> Baud;

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


	veci arm_Torque, arm_Torque_Direction;
	veci gripper_Torque, gripper_Torque_Direction;

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

	cout << "Press any key to start Grasping Motion" << endl;
	_getch();
	cout << "start" << endl;
	cout << "Current Calculated Angle is" << endl;

	while (MotionPlayer.Get_CurrentTime() < ArmTrajectory.GetMotionTotalTime())
	{
		// Torque 
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

	ArmComm.Arm_Torque_Off();
	ArmComm.Gripper_Torque_Off();
	ArmComm.DXL_Close();
}