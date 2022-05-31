#ifndef __PRO_ARM_COMM_WIN_H_
#define __PRO_ARM_COMM_WIN_H_

#include "ARMSDK_Math.h"
#include "RobotisLib/dynamixel.h"


namespace armsdk
{
	class Pro_Arm_Comm_Win
	{
		veci ARM_IDLIST;
		veci Gripper_IDLIST;
		int mPortnum;
		int mBaudnum;
		BulkData bd[256];
		BulkData *pbd[256];

		SerialPort sp, *Port;
	public:
		//General
		Pro_Arm_Comm_Win();
		Pro_Arm_Comm_Win(int portnum, int baudnum);
		~Pro_Arm_Comm_Win(){  }

		void DXL_Set_Init_Param(int portnum, int baudnum);
		int DXL_Open();
		SerialPort* DXL_Get_Port(void);
		void DXL_Close(void);

		//Robot Arm
		void Arm_ID_Setup(veci Arm_ID_LIST);
		int Arm_Torque_On(void);
		int Arm_Torque_Off(void);

		int Arm_Set_JointPosition(veci position);
		int Arm_Set_JointVelocity(veci velocity);
		int Arm_Set_JointVelocity(int velocity);
		int Arm_Set_JointAcceleration(veci accel);
		int Arm_Set_JointAcceleration(int accel);

		int Arm_Set_Position_PID_Gain(int P_Gain, int I_Gain, int D_Gain);
		int Arm_Set_Position_PID_Gain(int id, int P_Gain, int I_Gain, int D_Gain, int* ErrorStatus);


		int Arm_Get_JointPosition(veci *position);
		int Arm_Get_JointCurrent(veci *torque);

		int Arm_LED_On(void);
		int Arm_LED_Off(void);
		int Arm_LED_On(int r, int g, int b);
		int Arm_Red_LED_On(void);
		int Arm_Green_LED_On(void);
		int Arm_Blue_LED_On(void);


		int Arm_Get_External_Data(int id, int *ex_data1, int *ex_data2, int *ex_data3, int *ex_data4, int* ErrorStatus);

		void Gripper_ID_Setup(veci Gripper_ID_List);

		//for MX106 Gripper
		int Gripper_Ping(void);
		int Gripper_Torque_On(void);
		int Gripper_Torque_Off(void);

		int Gripper_Get_Joint_Value(veci *value);
		int Gripper_Set_Joint_Value(veci value);
		int Gripper_Set_Position_PID_Gain(int P, int I, int D);

		int Gripper_Get_Load_Value(veci *value);

		//for Pro Gripper
		int GripperPro_Ping(void);
		int GripperPro_Torque_On(void);
		int GripperPro_Torque_Off(void);

		int GripperPro_Get_Joint_Value(veci *value);
		int GripperPro_Set_Joint_Value(veci value);
	};
}
#endif