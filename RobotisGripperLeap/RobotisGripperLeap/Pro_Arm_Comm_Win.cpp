#include <conio.h>
#include <iostream>
#include "Pro_Arm_Comm_Win.h"
#include "RobotisLib\PRO_CTRL_TABLE.h"
#include "RobotisLib\MX_CTRL_TABLE.h"

namespace armsdk
{
	Pro_Arm_Comm_Win::Pro_Arm_Comm_Win()
	{
		sp.fByteTransferTime = 0;
		sp.dPacketStartTime = 0;
		sp.fPacketWaitTime = 0;
		sp.hComm = 0;
		sp.iBusUsing = 0;

		Port = &sp;

		for (unsigned int i = 0; i <256; i++)
		{
			pbd[i] = &bd[i];
			pbd[i]->pucTable = 0;
		}
	}

	Pro_Arm_Comm_Win::Pro_Arm_Comm_Win(int portnum, int baudnum)
	{
		sp.fByteTransferTime = 0;
		sp.dPacketStartTime = 0;
		sp.fPacketWaitTime = 0;
		sp.hComm = 0;
		sp.iBusUsing = 0;

		Port = &sp;

		for (unsigned int i = 0; i <256; i++)
		{
			pbd[i] = &bd[i];
			pbd[i]->pucTable = 0;
		}

		mPortnum = portnum; 	mBaudnum = baudnum;
	}

	void Pro_Arm_Comm_Win::DXL_Set_Init_Param(int portnum, int baudnum)
	{
		mPortnum = portnum;
		mBaudnum = baudnum;
	}

	SerialPort* Pro_Arm_Comm_Win::DXL_Get_Port(void)
	{
		return Port;
	}

	int Pro_Arm_Comm_Win::DXL_Open()
	{
		return dxl_initialize(Port, mPortnum, mBaudnum);
		if (dxl_initialize(Port, mPortnum, mBaudnum) == 0)
		{
			printf("Failed to open USB2Dynamixel(COM%d)!\n", mPortnum);
			return 0;
		}
		else
		{
			printf("Succeed to open USB2Dynamixel(COM%d)!\n", mPortnum);
			return 1;
		}
	}

	void Pro_Arm_Comm_Win::DXL_Close(void)
	{
		dxl_terminate(Port);
		printf("DXL is terminated\n");
	}

	void Pro_Arm_Comm_Win::Arm_ID_Setup(veci Arm_ID_List)
	{
		ARM_IDLIST.resize(Arm_ID_List.size());
		ARM_IDLIST = Arm_ID_List;
	}

	int Pro_Arm_Comm_Win::Arm_Set_JointPosition(veci position)
	{
		if (position.size() != ARM_IDLIST.size())
			return COMM_TXFAIL;

		unsigned char* param = new unsigned char[5 * position.size()];
		for (int i = 0; i < position.size(); i++)
		{
			param[i * 5] = (unsigned char)ARM_IDLIST[i];
			param[i * 5 + 1] = DXL_LOBYTE(DXL_LOWORD(position[i]));
			param[i * 5 + 2] = DXL_HIBYTE(DXL_LOWORD(position[i]));
			param[i * 5 + 3] = DXL_LOBYTE(DXL_HIWORD(position[i]));
			param[i * 5 + 4] = DXL_HIBYTE(DXL_HIWORD(position[i]));
		}
		int b = dxl_sync_write(Port, Pro::P_GOAL_POSITION_LL, 4, param, 5 * position.size());
		delete[] param;
		return b;
	}

	int Pro_Arm_Comm_Win::Arm_Set_JointVelocity(veci velocity)
	{
		if (velocity.size() != ARM_IDLIST.size())
			return COMM_TXFAIL;

		unsigned char* param = new unsigned char[5 * velocity.size()];
		for (int i = 0; i < velocity.size(); i++)
		{
			param[i * 5] = (unsigned char)ARM_IDLIST[i];
			param[i * 5 + 1] = DXL_LOBYTE(DXL_LOWORD(velocity[i]));
			param[i * 5 + 2] = DXL_HIBYTE(DXL_LOWORD(velocity[i]));
			param[i * 5 + 3] = DXL_LOBYTE(DXL_HIWORD(velocity[i]));
			param[i * 5 + 4] = DXL_HIBYTE(DXL_HIWORD(velocity[i]));
		}
		int b = dxl_sync_write(Port, Pro::P_GOAL_VELOCITY_LL, 4, param, 5 * velocity.size());
		delete[] param;
		return b;
	}

	int Pro_Arm_Comm_Win::Arm_Set_JointVelocity(int velocity)
	{
		unsigned char* param = new unsigned char[5 * ARM_IDLIST.size()];
		for (int i = 0; i < ARM_IDLIST.size(); i++)
		{
			param[i * 5] = (unsigned char)ARM_IDLIST[i];
			param[i * 5 + 1] = DXL_LOBYTE(DXL_LOWORD(velocity));
			param[i * 5 + 2] = DXL_HIBYTE(DXL_LOWORD(velocity));
			param[i * 5 + 3] = DXL_LOBYTE(DXL_HIWORD(velocity));
			param[i * 5 + 4] = DXL_HIBYTE(DXL_HIWORD(velocity));
		}
		int b = dxl_sync_write(Port, Pro::P_GOAL_VELOCITY_LL, 4, param, 5 * ARM_IDLIST.size());
		delete[] param;
		return b;
	}

	int Pro_Arm_Comm_Win::Arm_Set_JointAcceleration(veci accel)
	{
		if (accel.size() != ARM_IDLIST.size())
			return COMM_TXFAIL;

		unsigned char* param = new unsigned char[5 * accel.size()];
		for (int i = 0; i < accel.size(); i++)
		{
			param[i * 5] = (unsigned char)ARM_IDLIST[i];
			param[i * 5 + 1] = DXL_LOBYTE(DXL_LOWORD(accel[i]));
			param[i * 5 + 2] = DXL_HIBYTE(DXL_LOWORD(accel[i]));
			param[i * 5 + 3] = DXL_LOBYTE(DXL_HIWORD(accel[i]));
			param[i * 5 + 4] = DXL_HIBYTE(DXL_HIWORD(accel[i]));
		}
		int b = dxl_sync_write(Port, Pro::P_GOAL_ACCELATION_LL, 4, param, 5 * accel.size());
		delete[] param;
		return b;
	}


	int Pro_Arm_Comm_Win::Arm_Set_JointAcceleration(int accel)
	{
		unsigned char* param = new unsigned char[5 * ARM_IDLIST.size()];
		for (int i = 0; i < ARM_IDLIST.size(); i++)
		{
			param[i * 5] = (unsigned char)ARM_IDLIST[i];
			param[i * 5 + 1] = DXL_LOBYTE(DXL_LOWORD(accel));
			param[i * 5 + 2] = DXL_HIBYTE(DXL_LOWORD(accel));
			param[i * 5 + 3] = DXL_LOBYTE(DXL_HIWORD(accel));
			param[i * 5 + 4] = DXL_HIBYTE(DXL_HIWORD(accel));
		}
		int b = dxl_sync_write(Port, Pro::P_GOAL_ACCELATION_LL, 4, param, 5 * ARM_IDLIST.size());
		delete[] param;
		return b;
	}


	int Pro_Arm_Comm_Win::Arm_Torque_On(void)
	{
		unsigned char* param = new unsigned char[2 * ARM_IDLIST.size()];
		for (int i = 0; i < ARM_IDLIST.size(); i++)
		{
			param[i * 2] = ARM_IDLIST[i];
			param[i * 2 + 1] = 1;
		}
		int b = dxl_sync_write(Port, Pro::P_TORQUE_ENABLE, 1, param, 2 * ARM_IDLIST.size());
		delete[]param;
		return b;
	}

	int Pro_Arm_Comm_Win::Arm_Torque_Off(void)
	{
		unsigned char* param = new unsigned char[2 * ARM_IDLIST.size()];
		for (int i = 0; i < ARM_IDLIST.size(); i++)
		{
			param[i * 2] = ARM_IDLIST[i];
			param[i * 2 + 1] = 0;
		}
		int b = dxl_sync_write(Port, Pro::P_TORQUE_ENABLE, 1, param, 2 * ARM_IDLIST.size());
		delete[]param;
		return b;
	}

	int Pro_Arm_Comm_Win::Arm_Get_JointPosition(veci *position)
	{
		if (position->size() != ARM_IDLIST.size())
			return COMM_TXFAIL;

		//int bCommResult = 0;
		//
		//bCommResult = dxl_sync_read(Port, Pro::P_PRESENT_POSITION_LL, 4, , position->size(), )

		//int joint_idx = 0;
		//unsigned char *param = new unsigned char[5*ARM_IDLIST.size()];
		//for( joint_idx = 0 ; joint_idx < ARM_IDLIST.size() ; joint_idx++ )
		//{
		//	param[joint_idx*5] = (unsigned char)ARM_IDLIST[i];
		//	param[joint_idx*5+1] = DXL_LOBYTE(Pro::P_PRESENT_POSITION_LL);
		//	param[joint_idx*5+2] = DXL_HIBYTE(Pro::P_PRESENT_POSITION_LL);
		//	param[joint_idx*5+3] = DXL_LOBYTE(4);
		//	param[joint_idx*5+4] = DXL_HIBYTE(4);
		//}

		int bCommResult = 0;
		unsigned int tempval = 0;
		int joint_idx = 0;

		unsigned char *param = new unsigned char[5 * ARM_IDLIST.size()];

		for (joint_idx = 0; joint_idx < ARM_IDLIST.size(); joint_idx++)
		{
			param[joint_idx * 5 + 0] = ARM_IDLIST[joint_idx];
			param[joint_idx * 5 + 1] = DXL_LOBYTE(Pro::P_PRESENT_POSITION_LL);
			param[joint_idx * 5 + 2] = DXL_HIBYTE(Pro::P_PRESENT_POSITION_LL);
			param[joint_idx * 5 + 3] = DXL_LOBYTE(4);
			param[joint_idx * 5 + 4] = DXL_HIBYTE(4);

			if (pbd[ARM_IDLIST[joint_idx]]->pucTable != 0) {
				delete pbd[ARM_IDLIST[joint_idx]]->pucTable;
				pbd[ARM_IDLIST[joint_idx]]->pucTable = 0;
			}
		}


		bCommResult = dxl_bulk_read(Port, param, 5 * ARM_IDLIST.size(), pbd);

		delete[]param;

		if (bCommResult != COMM_RXSUCCESS) {
			return bCommResult;
		}
		else {
			for (joint_idx = 0; joint_idx < ARM_IDLIST.size(); joint_idx++)
			{
				bCommResult = dxl_get_bulk_dword(pbd, ARM_IDLIST[joint_idx], Pro::P_PRESENT_POSITION_LL, &tempval);
				(*position)(joint_idx) = (int)tempval;
				if (bCommResult != 1)
					continue;
			}

			return bCommResult;
		}

		//unsigned int _temp_val;
		//int Err, CommResult;

		//for(int joint_idx = 0; joint_idx < ARM_IDLIST.size() ; joint_idx ++)
		//{
		//	for(int CommAttempts = 0; CommAttempts < 10; CommAttempts++)
		//	{
		//		CommResult = dxl_read_dword(Port, (int)ARM_IDLIST(joint_idx), Pro::P_PRESENT_POSITION_LL, &_temp_val, &Err);
		//		if(CommResult == COMM_RXSUCCESS)
		//			break;
		//	}
		//	if(CommResult != COMM_RXSUCCESS)
		//		return CommResult;
		//	(*position)(joint_idx) = (int)_temp_val;
		//}

		//return CommResult;
	}

	int Pro_Arm_Comm_Win::Arm_Get_JointCurrent(veci *torque)
	{
		int b;
		int readval;

		if (torque->size() != ARM_IDLIST.size())
			return COMM_TXFAIL;


		for (int i = 0; i < ARM_IDLIST.size(); i++)
		{
			b = dxl_read_word(Port, ARM_IDLIST[i], Pro::P_PRESENT_CURRENT_L, &readval, 0);
			if (b != COMM_RXSUCCESS)
				return b;

			(*torque)(i) = (short int)readval;
		}
		//short int current;
		//torque->resize(ARM_IDLIST.size());
		//
		//int i = 0;
		//unsigned char *param = new unsigned char[5*ARM_IDLIST.size()];
		//for( i = 0 ; i < ARM_IDLIST.size() ; i++ )
		//{
		//	param[i*5] = (unsigned char)ARM_IDLIST[i];
		//	param[i*5+1] = DXL_LOBYTE(NX::P_PRESENT_CURRENT_L);
		//	param[i*5+2] = DXL_HIBYTE(NX::P_PRESENT_CURRENT_L);
		//	param[i*5+3] = DXL_LOBYTE(2);
		//	param[i*5+4] = DXL_HIBYTE(2);
		//}

		//b = dxl_bulk_read(Port, param, 5*ARM_IDLIST.size(), pbd);
		//delete[] param;

		//if( b != 1)
		//{

		//	return b;
		//}

		//for(i = 0 ; i < ARM_IDLIST.size() ; i++)
		//{
		//	b = dxl_get_bulk_word(pbd, ARM_IDLIST[i], NX::P_PRESENT_CURRENT_L, &readval);
		//	(*torque)(i) = (short int)readval;
		//	if( b != 1 )
		//		return b;
		//}

		return b;
	}


	int Pro_Arm_Comm_Win::Arm_Set_Position_PID_Gain(int P, int I, int D)
	{
		unsigned char* param = new unsigned char[7 * ARM_IDLIST.size()];
		for (int i = 0; i < ARM_IDLIST.size(); i++)
		{
			param[i * 7] = (unsigned char)ARM_IDLIST[i];
			param[i * 7 + 1] = DXL_LOBYTE(D);
			param[i * 7 + 2] = DXL_HIBYTE(D);
			param[i * 7 + 3] = DXL_LOBYTE(I);
			param[i * 7 + 4] = DXL_HIBYTE(I);
			param[i * 7 + 5] = DXL_LOBYTE(P);
			param[i * 7 + 6] = DXL_HIBYTE(P);
		}
		int b = dxl_sync_write(Port, Pro::P_POSITION_D_GAIN_L, 6, param, 7 * ARM_IDLIST.size());
		delete[]param;
		return b;
	}

	int Pro_Arm_Comm_Win::Arm_Set_Position_PID_Gain(int id, int P_Gain, int I_Gain, int D_Gain, int* ErrorStatus)
	{
		unsigned char param[6];

		param[0] = DXL_LOBYTE(D_Gain);
		param[1] = DXL_HIBYTE(D_Gain);
		param[2] = DXL_LOBYTE(I_Gain);
		param[3] = DXL_HIBYTE(I_Gain);
		param[4] = DXL_LOBYTE(P_Gain);
		param[5] = DXL_HIBYTE(P_Gain);

		return dxl_write(Port, id, Pro::P_POSITION_D_GAIN_L, 6, param, ErrorStatus);
	}

	int Pro_Arm_Comm_Win::Arm_LED_On(void)
	{
		unsigned char* param = new unsigned char[4 * ARM_IDLIST.size()];
		for (int i = 0; i < ARM_IDLIST.size(); i++)
		{
			param[i * 4 + 0] = ARM_IDLIST[i];
			param[i * 4 + 1] = 255;
			param[i * 4 + 2] = 255;
			param[i * 4 + 3] = 255;
		}
		int b = dxl_sync_write(Port, Pro::P_LED_RED, 3, param, 4 * ARM_IDLIST.size());
		delete[]param;
		return b;
	}

	int Pro_Arm_Comm_Win::Arm_LED_Off(void)
	{
		unsigned char* param = new unsigned char[4 * ARM_IDLIST.size()];
		for (int i = 0; i < ARM_IDLIST.size(); i++)
		{
			param[i * 4 + 0] = ARM_IDLIST[i];
			param[i * 4 + 1] = 0;
			param[i * 4 + 2] = 0;
			param[i * 4 + 3] = 0;
		}
		int b = dxl_sync_write(Port, Pro::P_LED_RED, 3, param, 4 * ARM_IDLIST.size());
		delete[]param;
		return b;
	}

	int Pro_Arm_Comm_Win::Arm_LED_On(int red, int green, int blue)
	{
		if (red>255)
			red = 255;

		if (green>255)
			green = 255;

		if (blue > 255)
			blue = 255;

		unsigned char* param = new unsigned char[4 * ARM_IDLIST.size()];
		for (int i = 0; i < ARM_IDLIST.size(); i++)
		{
			param[i * 4 + 0] = ARM_IDLIST[i];
			param[i * 4 + 1] = red;
			param[i * 4 + 2] = green;
			param[i * 4 + 3] = blue;
		}
		int b = dxl_sync_write(Port, Pro::P_LED_RED, 3, param, 4 * ARM_IDLIST.size());
		delete[]param;
		return b;
	}

	int Pro_Arm_Comm_Win::Arm_Red_LED_On(void)
	{
		unsigned char* param = new unsigned char[2 * ARM_IDLIST.size()];
		for (int i = 0; i < ARM_IDLIST.size(); i++)
		{
			param[i * 2 + 0] = ARM_IDLIST[i];
			param[i * 2 + 1] = 255;
		}
		int b = dxl_sync_write(Port, Pro::P_LED_RED, 1, param, 2 * ARM_IDLIST.size());
		delete[]param;
		return b;
	}


	int Pro_Arm_Comm_Win::Arm_Green_LED_On(void)
	{
		unsigned char* param = new unsigned char[2 * ARM_IDLIST.size()];
		for (int i = 0; i < ARM_IDLIST.size(); i++)
		{
			param[i * 2 + 0] = ARM_IDLIST[i];
			param[i * 2 + 1] = 255;
		}
		int b = dxl_sync_write(Port, Pro::P_LED_GREEN, 1, param, 2 * ARM_IDLIST.size());
		delete[]param;
		return b;
	}


	int Pro_Arm_Comm_Win::Arm_Blue_LED_On(void)
	{
		unsigned char* param = new unsigned char[2 * ARM_IDLIST.size()];
		for (int i = 0; i < ARM_IDLIST.size(); i++)
		{
			param[i * 2 + 0] = ARM_IDLIST[i];
			param[i * 2 + 1] = 255;
		}
		int b = dxl_sync_write(Port, Pro::P_LED_BLUE, 1, param, 2 * ARM_IDLIST.size());
		delete[]param;
		return b;
	}


	int Pro_Arm_Comm_Win::Arm_Get_External_Data(int id, int *ex_data1, int *ex_data2, int *ex_data3, int *ex_data4, int* ErrorStatus)
	{
		int bCommResult = 0;
		unsigned char read_data[8];

		bCommResult = dxl_read(Port, id, Pro::P_EXTERNAL_PORT_DATA_1_L, 8, read_data, ErrorStatus);

		if (bCommResult != COMM_RXSUCCESS)
			return bCommResult;
		else
		{
			*ex_data1 = DXL_MAKEWORD(read_data[0], read_data[1]);
			*ex_data2 = DXL_MAKEWORD(read_data[2], read_data[3]);
			*ex_data3 = DXL_MAKEWORD(read_data[4], read_data[5]);
			*ex_data4 = DXL_MAKEWORD(read_data[6], read_data[7]);
			return bCommResult;
		}
	}


	//Gripper
	void Pro_Arm_Comm_Win::Gripper_ID_Setup(veci Gripper_ID_List)
	{
		Gripper_IDLIST = Gripper_ID_List;
	}

	int Pro_Arm_Comm_Win::Gripper_Ping()
	{
		PingData Gripper_PingData;
		int CommResult = COMM_RXSUCCESS;
		if (Gripper_IDLIST.size() == 0)
			return 0;
		else
		{
			for (int i = 0; i < Gripper_IDLIST.size(); i++)
			{
				CommResult = dxl_ping(Port, Gripper_IDLIST(i), &Gripper_PingData, 0);
				if (CommResult != COMM_RXSUCCESS)
					return 0;
			}
			return 1;
		}
	}

	int  Pro_Arm_Comm_Win::Gripper_Torque_On(void)
	{
		unsigned char* param = new unsigned char[2 * Gripper_IDLIST.size()];
		for (int i = 0; i < Gripper_IDLIST.size(); i++)
		{
			param[i * 2] = Gripper_IDLIST[i];
			param[i * 2 + 1] = 1;
		}
		int b = dxl_sync_write(Port, MX::P_TORQUE_ENABLE, 1, param, 2 * Gripper_IDLIST.size());
		delete[]param;
		return b;
	}

	int  Pro_Arm_Comm_Win::Gripper_Torque_Off(void)
	{
		unsigned char* param = new unsigned char[2 * Gripper_IDLIST.size()];
		for (int i = 0; i < Gripper_IDLIST.size(); i++)
		{
			param[i * 2] = Gripper_IDLIST[i];
			param[i * 2 + 1] = 0;
		}
		int b = dxl_sync_write(Port, MX::P_TORQUE_ENABLE, 1, param, 2 * Gripper_IDLIST.size());
		delete[]param;
		return b;
	}

	int  Pro_Arm_Comm_Win::Gripper_Get_Joint_Value(veci *value)
	{
		int _temp_val;
		int Err, CommResult;

		for (int joint_idx = 0; joint_idx < Gripper_IDLIST.size(); joint_idx++)
		{
			for (int CommAttempts = 0; CommAttempts < 10; CommAttempts++)
			{
				CommResult = dxl_read_word(Port, (int)Gripper_IDLIST(joint_idx), MX::P_PRESENT_POSITION_L, &_temp_val, &Err);
				if (CommResult == COMM_RXSUCCESS)
					break;
			}
			if (CommResult != COMM_RXSUCCESS)
				return CommResult;

			(*value)(joint_idx) = _temp_val;
		}

		return CommResult;
	}

	int  Pro_Arm_Comm_Win::Gripper_Set_Joint_Value(veci value)
	{
		if (value.size() != Gripper_IDLIST.size())
			return 0;

		unsigned char* param = new unsigned char[3 * value.size()];
		for (int i = 0; i < value.size(); i++)
		{
			param[i * 3] = (unsigned char)Gripper_IDLIST[i];
			param[i * 3 + 1] = DXL_LOBYTE((value[i]));
			param[i * 3 + 2] = DXL_HIBYTE((value[i]));
		}
		int b = dxl_sync_write(Port, MX::P_GOAL_POSITION_L, 2, param, 3 * value.size());
		delete[] param;
		return b;
	}

	int Pro_Arm_Comm_Win::Gripper_Set_Position_PID_Gain(int P, int I, int D)
	{
		unsigned char* param = new unsigned char[7 * Gripper_IDLIST.size()];
		for (int i = 0; i < Gripper_IDLIST.size(); i++)
		{
			param[i * 4] = (unsigned char)Gripper_IDLIST[i];
			param[i * 4 + 1] = P;
			param[i * 4 + 2] = I;
			param[i * 4 + 3] = D;
		}
		int b = dxl_sync_write(Port, MX::P_P_GAIN, 3, param, 7 * Gripper_IDLIST.size());
		delete[]param;
		return b;
	}

	int Pro_Arm_Comm_Win::Gripper_Get_Load_Value(veci *value){
		int _temp_val;
		int Err, CommResult;

		for (int joint_idx = 0; joint_idx < Gripper_IDLIST.size(); joint_idx++)
		{
			for (int CommAttempts = 0; CommAttempts < 10; CommAttempts++)
			{
				CommResult = dxl_read_word(Port, (int)Gripper_IDLIST(joint_idx), MX::P_PRESENT_LOAD_L, &_temp_val, &Err);
				if (CommResult == COMM_RXSUCCESS)
					break;
			}
			if (CommResult != COMM_RXSUCCESS)
				return CommResult;

			(*value)(joint_idx) = _temp_val;
		}

		return CommResult;
	}
	// Gripper Pro 
	int Pro_Arm_Comm_Win::GripperPro_Ping(void)
	{
		PingData Gripper_PingData;
		int CommResult = COMM_TXFAIL;
		if (Gripper_IDLIST.size() == 0)
			return CommResult;
		else
		{
			for (int i = 0; i < Gripper_IDLIST.size(); i++)
			{
				CommResult = dxl_ping(Port, Gripper_IDLIST(i), &Gripper_PingData, 0);
				if (CommResult != COMM_RXSUCCESS)
					return COMM_RXTIMEOUT;
			}
			return 1;
		}
	}

	int Pro_Arm_Comm_Win::GripperPro_Torque_On(void)
	{
		unsigned char* param = new unsigned char[2 * Gripper_IDLIST.size()];
		for (int i = 0; i < Gripper_IDLIST.size(); i++)
		{
			param[i * 2] = Gripper_IDLIST[i];
			param[i * 2 + 1] = 1;
		}
		int b = dxl_sync_write(Port, Pro::P_TORQUE_ENABLE, 1, param, 2 * Gripper_IDLIST.size());
		delete[]param;
		return b;
	}

	int Pro_Arm_Comm_Win::GripperPro_Torque_Off(void)
	{
		unsigned char* param = new unsigned char[2 * Gripper_IDLIST.size()];
		for (int i = 0; i < Gripper_IDLIST.size(); i++)
		{
			param[i * 2] = Gripper_IDLIST[i];
			param[i * 2 + 1] = 0;
		}
		int b = dxl_sync_write(Port, Pro::P_TORQUE_ENABLE, 1, param, 2 * Gripper_IDLIST.size());
		delete[]param;
		return b;
	}

	int Pro_Arm_Comm_Win::GripperPro_Get_Joint_Value(veci *value)
	{
		unsigned int _temp_val;
		int Err, CommResult;

		for (int joint_idx = 0; joint_idx < Gripper_IDLIST.size(); joint_idx++)
		{
			for (int CommAttempts = 0; CommAttempts < 10; CommAttempts++)
			{
				CommResult = dxl_read_dword(Port, (int)Gripper_IDLIST(joint_idx), Pro::P_PRESENT_POSITION_LL, &_temp_val, &Err);
				if (CommResult == COMM_RXSUCCESS)
					break;
			}
			if (CommResult != COMM_RXSUCCESS)
				return CommResult;

			(*value)(joint_idx) = (int)_temp_val;
		}

		return CommResult;
	}

	int Pro_Arm_Comm_Win::GripperPro_Set_Joint_Value(veci value)
	{
		if (value.size() != Gripper_IDLIST.size())
			return 0;

		unsigned char* param = new unsigned char[5 * value.size()];
		for (int i = 0; i < value.size(); i++)
		{
			param[i * 5 + 0] = (unsigned char)Gripper_IDLIST[i];
			param[i * 5 + 1] = DXL_LOBYTE(DXL_LOWORD(value[i]));
			param[i * 5 + 2] = DXL_HIBYTE(DXL_LOWORD(value[i]));
			param[i * 5 + 2] = DXL_LOBYTE(DXL_HIWORD(value[i]));
			param[i * 5 + 2] = DXL_HIBYTE(DXL_HIWORD(value[i]));
		}
		int b = dxl_sync_write(Port, Pro::P_GOAL_POSITION_LL, 4, param, 5 * value.size());
		delete[] param;
		return b;
	}

}