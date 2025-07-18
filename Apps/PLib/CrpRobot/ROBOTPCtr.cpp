#include "stdafx.h"
#include "ROBOTPCtr.h"

CROBOTPCtr::CROBOTPCtr()
{
	Obj_Modbus = new TCP_Socket_CRP();
	Obj_FTP_cmd = new TCP_Socket_CRP();
	Obj_FTP_date = new TCP_Socket_CRP();
	Obj_Track = new TCP_Socket_CRP();
	sendBuf = new vector<uint16_t>();
	Number_a = 1;
	Modbus_Flag = 0;//modbusTCP里的事务标识符
	Modbus_rcvBuf = new map<int, vector<uint8_t>>();
	m_mutex = CreateMutex(NULL, FALSE, "Mutex");     //创建一把锁
	/*********************************工具坐标数据************************************************************/
	//创建一个10个工具坐标数组
	ToolArray = (double**)malloc(sizeof(double *) * 10);//创建10个工具
	for (size_t i = 0; i < 10; i++)
	{
		ToolArray[i] = (double*)malloc(sizeof(double) * 6);
	}
	for (size_t j = 0; j < 10; j++)
	{
		for (size_t i = 0; i < 6; i++)
		{
			ToolArray[j][i] = 0;
		}
	}
	/*************************************用户坐标数据**************************************************************/
	//为后续设置用户坐标，转换用户坐标留接口
	//创建一个10个用户坐标数组
	UserArray = (double**)malloc(sizeof(double*) * 10);
	for (size_t i = 0; i < 10; i++)
	{
		UserArray[i] = (double*)malloc(sizeof(double) * 6);
	}
	//用户0赋值
	for (size_t j = 0; j < 10; j++)
	{
		for (size_t i = 0; i < 6; i++)
		{
			UserArray[0][i] = 0;
		}
	}
	/***************************************************************************************************************/
}


CROBOTPCtr::~CROBOTPCtr()
{
	delete Obj_Modbus;
	delete Obj_FTP_cmd;
	delete Obj_FTP_date;
	delete Modbus_rcvBuf;
	delete sendBuf;
	//
	for (size_t i = 0; i < 10; i++)
	{
		free(ToolArray[i]);
	}
	free(ToolArray);
	//
	for (size_t i = 0; i < 10; i++)
	{
		free(UserArray[i]);
	}
	free(UserArray);

	//delete m_mutex;
}

void CROBOTPCtr::initApp()
{
	time_t now = time(NULL);
	while (1) {
		if (!Obj_FTP_cmd->m_connected) {
			while (ToolArray[1][0] == 0)
			{
				initFTP_cmd("192.168.0.100", 21, "Xi_Robot", "123456");//21就是ftp的默认命令端口
				initModbusTCP("192.168.0.100", 8001);
				Sleep(50);
				initToolInfo();
				initUserInfo();
			}
			break;
		}
		if (time(NULL) - now > 5) { 
			break;
		}
	}
	time_t after = time(NULL);
}

void CROBOTPCtr::keepAlive()
{
	int res = Read_M_Number_status(21);
	while (res < 0 ) {
		initFTP_cmd("192.168.0.100", 21, "Xi_Robot", "123456");//21就是ftp的默认命令端口
		initModbusTCP("192.168.0.100", 8001);
		Sleep(50);
		res = Read_M_Number_status(21);
	}
}

int CROBOTPCtr::initModbusTCP(char * ip, u_short Prot, int flag)
{
	m_ip = ip;
	m_prot = Prot;
	m_flag = flag;
	Obj_Modbus->m_FTPClient__ = 0;
	Obj_Modbus->m_ModbusTCPClient__ = 1;
	Obj_Modbus->TCP_init(m_ip, m_prot, m_flag);
	Obj_Modbus->SetCallBackRcvData(Modbus_Rcv, this);
	return Obj_Modbus->startSocket();
}

int CROBOTPCtr::initFTP_cmd(char * m_ip, u_short Prot, char* name, char*  password, int flag)
{
	FTPm_ip = m_ip;
	FTPm_prot = Prot;
	FTPm_flag = flag;
	FTPname = name;
	FTPpassword = password;
	Obj_FTP_cmd->m_FTPClient__ = 1;
	Obj_FTP_cmd->m_ModbusTCPClient__ = 0;
	Obj_FTP_cmd->TCP_init(FTPm_ip, FTPm_prot, FTPm_flag);
	Obj_FTP_cmd->SetCallBackRcvData(FTP_Rcv, this);
	return Obj_FTP_cmd->startSocket();
}



int CROBOTPCtr::initTrackTCP(u_short Port)
{
	int res = Obj_Track->track(Port);
	return res;
}

int CROBOTPCtr::initToolUserDate(CRP_T00L Tool[], int ToolCount, CRP_USERCOOR UserCoor[], int UserCount)
{
	//工具赋值
	int toolCount_i = ToolCount > 10 ? ToolCount : 10;
	for (size_t i = 0; i < toolCount_i; i++)
	{
		ToolArray[i][0] = Tool[i].X;
		ToolArray[i][1] = Tool[i].Y;
		ToolArray[i][2] = Tool[i].Z;
		ToolArray[i][3] = Tool[i].A;
		ToolArray[i][4] = Tool[i].B;
		ToolArray[i][5] = Tool[i].C;
	}
	//用户0赋值
	int userCount_i = UserCount > 10 ? UserCount : 10;
	for (size_t i = 0; i < userCount_i; i++)
	{
		UserArray[i][0] = UserCoor[i].X;
		UserArray[i][1] = UserCoor[i].Y;
		UserArray[i][2] = UserCoor[i].Z;
		UserArray[i][3] = UserCoor[i].A;
		UserArray[i][4] = UserCoor[i].B;
		UserArray[i][5] = UserCoor[i].C;
	}
	return 0;
}

template<typename T> bool CROBOTPCtr::pose2ora(T const(&pose)[9], T & a, T & b, T & c)
{
	/*double PI = 3.1415926;
	b = atan2((0 - pose[6]), sqrt(pose[0] * pose[0] + pose[3] * pose[3]));

	if (almostEqual(b, PI / 2, 6) || almostEqual(b, -PI / 2, 6))
	{
		c = 0.0;
		if (almostEqual(b, PI / 2, 6))
			a = atan2(pose[1], pose[4]);
		else
			a = -atan2(pose[1], pose[4]);
	}

	if (isZero(cos(b)))
	{
		return false;
	}

	c = atan2(pose[3] / cos(b), pose[0] / cos(b));
	a = atan2(pose[7] / cos(b), pose[8] / cos(b));
	return true;*/
	//double PI = 3.14159265358979323846; //M_PI
	b = atan2((0 - pose[6]), sqrt(pose[0] * pose[0] + pose[3] * pose[3]));

	if (almostEqual(b, PI / 2, 6) || almostEqual(b, -PI / 2, 6))
	{
		c = 0.0;
		if (almostEqual(b, PI / 2, 6))
			a = atan2(pose[1], pose[4]);
		else
			a = -atan2(pose[1], pose[4]);
	}

	if (isZero(cos(b)))
	{
		return false;
	}

	c = atan2(pose[3] / cos(b), pose[0] / cos(b));
	a = atan2(pose[7] / cos(b), pose[8] / cos(b));
	return true;
}

int CROBOTPCtr::Write_M_Number_status(int MNumber, bool status)
{
	bool result = false;
	sendBuf->clear();
	int addr = 4096 + MNumber;
	sendBuf->push_back(addr);
	status ? sendBuf->push_back(1) : sendBuf->push_back(0);
	int flag = ModbusTCP(Write_coil_bite_register, sendBuf);
	if (wait(flag, 1000) != 0)return -1;
	map<int, vector<uint8_t>>::iterator it = Modbus_rcvBuf->find(flag);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = 2;
	result = it->second.at(11) == 0xff;
	if (result != status) return -1;
	ReleaseMutex(m_mutex);
	return result;
}
int CROBOTPCtr::Read_X_Number_status(int XNumber)
{
	int result = 0;
	try
	{
		sendBuf->clear();
		sendBuf->push_back(XNumber);
		sendBuf->push_back(1);
		int cmd = Read_Input_bite_register;
		int flag = ModbusTCP(cmd, sendBuf);
		if (wait(flag, 1000) != 0)return -1;
		map<int, vector<uint8_t>>::iterator it = Modbus_rcvBuf->find(flag);
		WaitForSingleObject(m_mutex, 5000);
		it->second.at(0) = 2;
		int len = it->second.at(9);
		if (len == 1)
		{
			result = it->second.at(10) & 0x01;
		}
		ReleaseMutex(m_mutex);
	}
	catch (...)
	{
		delete sendBuf;
		sendBuf = new vector<uint16_t>();
		Read_X_Number_status(XNumber);
	}
	
	return result;
}
int CROBOTPCtr::Read_GI_Number_status(int GINumber)
{
	int result = 0;
	try
	{
		sendBuf->clear();
		sendBuf->push_back(GINumber);
		sendBuf->push_back(1);
		int cmd = Read_retain_word_register;
		int flag = ModbusTCP(cmd, sendBuf);
		if (wait(flag, 1000) != 0) {
			result = -1;
		}
		else {
			map<int, vector<uint8_t>>::iterator it = Modbus_rcvBuf->find(flag);
			WaitForSingleObject(m_mutex, 5000);
			it->second.at(0) = 2;
			int len = it->second.at(9) / 2;
			uint16_t data = 0, data_1, data_2;
			if (len == 1)
			{
				data_1 = it->second.at(10);
				data_2 = it->second.at(11);
			}
			//result = data_1*0x100+data_2;
			result = (data_1 << 8) | data_2;
			ReleaseMutex(m_mutex);
		}
	}
	catch (...)
	{
		delete sendBuf;
		sendBuf = new vector<uint16_t>();
		Read_GI_Number_status(GINumber);//出错整理好数据重新开始
	}

	return result;
}
int CROBOTPCtr::Write_GI_Number_status(int GINumber, int date)
{
	bool result = false;
	sendBuf->clear();
	sendBuf->push_back(GINumber);
	sendBuf->push_back(date);
	int cmd = Write_retain_word_register;
	int flag = ModbusTCP(cmd, sendBuf);
	if (wait(flag, 1000) != 0)return -1;
	map<int, vector<uint8_t>>::iterator it = Modbus_rcvBuf->find(flag);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = 2;
	uint16_t data_result = (it->second.at(11) << 8) | (it->second.at(12));
	result = data_result == date;
	ReleaseMutex(m_mutex);
	return result;
}


int CROBOTPCtr::Read_Y_Number_status(int YNumber)
{
	int result[3] = { 0 };
	int weight[2] = { 0 };
	int ans = 0;
	try
	{
		for (int i = 0; i < sizeof(result) / sizeof(int); i++)
		{
			sendBuf->clear(); // vector<uint16_t> *sendBuf
			sendBuf->push_back(YNumber);
			sendBuf->push_back(1); //1代表读1位
			int flag = ModbusTCP(Read_coil_bite_register, sendBuf); //返回当前请求事务标识符
			if (wait(flag, 1000) != 0)return -1;//没有这个请求就不响应？
			map<int, vector<uint8_t>>::iterator it = Modbus_rcvBuf->find(flag); //key为状态标识符，val输入缓冲区
			WaitForSingleObject(m_mutex, 5000); //等待之前的线程工作完毕再拿到互斥体权限
			it->second.at(0) = 2;
			int len = it->second.at(9);
			if (len == 1)
			{
				result[i] = it->second.at(10) & 0x01; //读取输入缓冲区数据
			}
			ReleaseMutex(m_mutex); //释放互斥体（同步）
		}
		for (int i = 0; i < sizeof(result) / sizeof(int); i++) {
			weight[result[i]]++;
		}
		ans = weight[0] > weight[1] ? 0 : 1;

	}
	catch (...)
	{
		delete sendBuf;
		sendBuf = new vector<uint16_t>();
		Read_Y_Number_status(YNumber);
	}

	return ans;
}


int CROBOTPCtr::Read_GP_Number_status(int GPNumber, double date[])
{
	int ans = 0;
	try
	{	
		vector<uint16_t> *sendBuf1 = new vector<uint16_t>();
		sendBuf1->push_back(0x3000 + GPNumber * 20);
		sendBuf1->push_back(10 * 2);
		int cmd = Read_retain_word_register;
		int flag = ModbusTCP(cmd, sendBuf1);
		if (wait(flag, 2000) != 0)return -1;
		map<int, vector<uint8_t>>::iterator it = Modbus_rcvBuf->find(flag);

		WaitForSingleObject(m_mutex, 5000);
		
		if (it == Modbus_rcvBuf->end())
		{
			ans = -1;
			return ans;
		}
		
		it->second.at(0) = 2;
		int len = it->second.at(9) / 2;
//		uint16_t  data_x, data_y;
		uint8_t  arr[4] = { 0,0,0,0 };
		if (len == 20)
		{
			int number = 10;

			for (int i = 0; i < 10; i++)
			{
				for (int j = 3; j >= 0; j--)
				{
					*(arr + j) = it->second.at(number++);
					//number++;
				}
				*(date + i) = *(float*)(arr);
			}
		}
		double data_RP[10] = { 0,0,0,0,0,0,0,0,0,0 };
		for (size_t i = 0; i < 10; i++)
		{
			*(data_RP + i) = *(date + i);
		}
		ReleaseMutex(m_mutex);
		
	}
	catch (...)
	{
		delete sendBuf;
		sendBuf = new vector<uint16_t>();
		Read_GP_Number_status(GPNumber, date);
	}
	
	return ans;
}

int CROBOTPCtr::getToolInfo(int toolNumber, double data[])
{
	if (toolNumber > 9) {
		return -1;
	}
	for (int i = 0; i < 6; i++) {
		data[i] = ToolArray[toolNumber][i];
	}
	return 1;
}

void CROBOTPCtr::MatrixTranspose(double RotationMatrix[3][3]) {
	double tmp = RotationMatrix[0][1];
	RotationMatrix[0][1] = RotationMatrix[1][0];
	RotationMatrix[1][0] = tmp;

	tmp = RotationMatrix[0][2];
	RotationMatrix[0][2] = RotationMatrix[2][0];
	RotationMatrix[2][0] = tmp;

	tmp = RotationMatrix[1][2];
	RotationMatrix[1][2] = RotationMatrix[2][1];
	RotationMatrix[2][1] = tmp;
}
int CROBOTPCtr::getUserInfo(int userNumber, double data[]) {
	if (userNumber > 9) {
		return -1;
	}
	for (int i = 0; i < 6; i++) {
		data[i] = UserArray[userNumber][i];
	}
	return 1;

}

int CROBOTPCtr::initToolInfo()
{
	const char name[] = "ToolCoord.txt";
	vector<string> info;
	int res = Obj_FTP_cmd->FTP_DownloadFileByCoverLocal("/file/Coord/ToolCoord.txt", name);
	if (res == 0) {
		ifstream ReadFile;
		int n = 0;
		string tmp;
		ReadFile.open("./file/ToolCoord.txt");//ios::in 表示以只读的方式读取文件
		if (ReadFile.fail())//文件打开失败:返回0
		{
			return 0;
		}
		else//文件存在
		{
			for (int i = 0; i < 10; i++)
			{
				info.swap(vector<string>()); //清空
				getline(ReadFile, tmp, '\n');
				string t;
				istringstream in(tmp);
				while (getline(in, t, ' ')) {	//这里单引号要注意
					info.push_back(t);
				}

				for (int j = 0; j < 6; j++) {
					ToolArray[i][j] = stold(info[116 + j]);
				}
			}
			ReadFile.close();

		}
	}
	return res;
}

int CROBOTPCtr::initUserInfo()
{
	const char name[] = "UseCoord.txt";
	vector<string> info;
	int res = Obj_FTP_cmd->FTP_DownloadFileByCoverLocal("/file/Coord/UseCoord.txt", name); //获取最新的用户文件
	if (res == 0) {
		ifstream ReadFile;
		string tmp;
		ReadFile.open("./file/UseCoord.txt");//ios::in 表示以只读的方式读取文件
		if (ReadFile.fail())//文件打开失败:返回0
		{
			return 0;
		}

		else//文件存在
		{
			/*************获取指定行的数据,按空格拆成若干数据，转成double数据并放入info************/
			for (int i = 0; i < 10; i++)
			{
				info.swap(vector<string>()); //清空
				getline(ReadFile, tmp, '\n');
				string t;
				istringstream in(tmp);
				while (getline(in, t, ' ')) {	//这里单引号要注意
					info.push_back(t);
				}
				/*************根据info中的旋转矩阵数据计算得到要显示的欧拉角数据************/
				double RotationMatrix[3][3] = { 0 };
				int matrixAddr = 41;
				int tmp = 0;
				for (int i = 0; i < 3; i++) {
					for (int j = 0; j < 3; j++) {
						RotationMatrix[i][j] = stold(info[41 + i * 3 + j + tmp]);//旋转矩阵起始下标是41
					}
					tmp++;
				}
				double oraA[3] = { 0 };
				RotationToEulerAngle(RotationMatrix, oraA); // 旋转矩阵转欧拉角
															/*************整理返回数据************/
				UserArray[i][0] = stold(info[44]);
				UserArray[i][1] = stold(info[48]);
				UserArray[i][2] = stold(info[52]);
				UserArray[i][3] = oraA[0];
				UserArray[i][4] = oraA[1];
				UserArray[i][5] = oraA[2];
			}
			ReadFile.close();

		}
	}
	return res;
}

int CROBOTPCtr::setUserInfo(int userNumber, double date[]) {
	ifstream ReadFile;
	ofstream location_out;
	string tmp;
	vector<string> info; //将一行数据转化为字符串数组存储(空格分隔)
//	double readUser[10];

	location_out.open("./file/NewUseCoord.txt", std::ios::out);
	ReadFile.open("./file/UseCoord.txt");//ios::in 表示以只读的方式读取文件
	if (ReadFile.fail())//文件打开失败
	{
		int res = Obj_FTP_cmd->FTP_DownloadFileByCoverLocal("/file/Coord/UseCoord.txt", "UseCoord.txt"); //获取最新的用户文件
		if (res < 0) return -1;
		else ReadFile.open("./file/UseCoord.txt");
	}
	else//文件存在
	{
		int lineNum = 0;
		while (getline(ReadFile, tmp, '\n'))
		{
			vector<string> nomalVector;
			if (lineNum != userNumber) {
				nomalVector.swap(vector<string>()); //清空
				string nomalLine;
				nomalLine.clear();
				istringstream in(tmp);
				while (getline(in, nomalLine, ' ')) {	//这里单引号要注意
					nomalVector.push_back(nomalLine);
				}
				//tmp += '\n';
				nomalVector[nomalVector.size() - 1] = "\r\r\n";
				nomalLine = "";
				for (int i = 0; i < nomalVector.size() - 1; i++) {
					nomalLine += nomalVector[i];
					nomalLine += ' ';
				}
				nomalLine += nomalVector[nomalVector.size() - 1];
				location_out << nomalLine;
			}
			if (lineNum == userNumber) {
				string t;
				istringstream in(tmp);
				while (getline(in, t, ' ')) {	//这里单引号要注意
					info.push_back(t);
				}

				info[69] = "\r\r\n";
				double ora[3] = { 0 };
				double RotationMatrix[3][3] = { 0 };
				ora[0] = date[3];
				ora[1] = date[4];
				ora[2] = date[5];
				EulerAngleToVector(ora, RotationMatrix); //将欧拉角转旋转矩阵
				MatrixTranspose(RotationMatrix); //将得到的旋转矩阵转置

				int matrixAddr = 41;
				int tmp = 0;
				stringstream ss;
				string str;

				for (int i = 0; i < 3; i++) {   //修改旋转矩阵
					for (int j = 0; j < 3; j++) {
						ss.str("");
						ss << RotationMatrix[i][j];
						str = ss.str();
						info[41 + i * 3 + j + tmp] = str;//旋转矩阵起始下标是41
					}
					tmp++;
				}

				for (int i = 0; i < 3; i++) {   //修改欧拉角
					ss.str("");
					ss << date[i];
					str = ss.str();
					info[44 + i * 4] = str;
				}
				str = "";
				for (int i = 0; i < 69; i++) {
					str += info[i];
					str += ' ';
				}
				str += info[69];
				location_out << str;
				//location_out << endl;
				//location_out << endl;
			}
			lineNum++;
		}
		location_out.close();
		ReadFile.close();
		remove("./file/UseCoord.txt");// == 0;
		rename("./file/NewUseCoord.txt", "./file/UseCoord.txt");
		bool res = Obj_FTP_cmd->FTP_SendFileToAddress("./file/UseCoord.txt", "file/Coord/");
	}
	return 0;
}

void CROBOTPCtr::EulerAngleToVector(double Angle[3], double Vector[3][3])
{
	double AN_TO_ARC = 3.141592653589793 / 180;
	double A = Angle[0] * AN_TO_ARC;
	double B = Angle[1] * AN_TO_ARC;
	double C = Angle[2] * AN_TO_ARC;
	double Nx = 0, Ny = 0, Nz = 0;
	double Ox = 0, Oy = 0, Oz = 0;
	double Ax = 0, Ay = 0, Az = 0;
	//物体在X轴上的投影的单位向量
	Nx = cos(C) * cos(B);
	Ny = sin(C) * cos(B);
	Nz = -sin(B);
	//物体在Y轴上的投影的单位向量
	Ox = cos(C) * sin(B) * sin(A) - sin(C) * cos(A);
	Oy = sin(C) * sin(B) * sin(A) + cos(C) * cos(A);
	Oz = cos(B)*sin(A);
	//物体在Z轴上的投影的单位向量
	Ax = cos(C) * sin(B) * cos(A) + sin(C) * sin(A);
	Ay = sin(C) * sin(B) * cos(A) - cos(C) * sin(A);
	Az = cos(B)*cos(A);

	Vector[0][0] = Nx;
	Vector[0][1] = Ny;
	Vector[0][2] = Nz;

	Vector[1][0] = Ox;
	Vector[1][1] = Oy;
	Vector[1][2] = Oz;

	Vector[2][0] = Ax;
	Vector[2][1] = Ay;
	Vector[2][2] = Az;

}

void CROBOTPCtr::RotationToEulerAngle(double RotationMatrix[3][3], double Angle[3])
{
	/*double tan_Rz = RotationMatrix[1][0] / RotationMatrix[0][0];
	double tan_Rx= RotationMatrix[2][1] / RotationMatrix[2][2];
	double sin_Ry =- RotationMatrix[2][0];

	Angle[0] = atan(tan_Rx)*57.29578049;
	Angle[1] = asin(sin_Ry)*57.29578049;
	Angle[2] = atan(tan_Rz)*57.29578049;*/
	double rotation[9] = { 0 };
	rotation[0] = RotationMatrix[0][0];
	rotation[1] = RotationMatrix[0][1];
	rotation[2] = RotationMatrix[0][2];
	rotation[3] = RotationMatrix[1][0];
	rotation[4] = RotationMatrix[1][1];
	rotation[5] = RotationMatrix[1][2];
	rotation[6] = RotationMatrix[2][0];
	rotation[7] = RotationMatrix[2][1];
	rotation[8] = RotationMatrix[2][2];

	double a, b, c;
	//pos2ora([NX,OX,AX,NY,OY,AY,NZ,OZ,AZ],a 弧度，b 弧度，c 弧度)
	bool iftrue = pose2ora(rotation, a, b, c);

	Angle[0] = a * 180 / 3.1415926;
	Angle[1] = b * 180 / 3.1415926;
	Angle[2] = c * 180 / 3.1415926;


}

void CROBOTPCtr::FlangeToTool_Pos(double F_pos[6], int ToolNumber, int UserNumber, double T_pos[6])
{
	/************************************当前法兰下的直角坐标**********************************************************/
	double angle[3] = { F_pos[3],F_pos[4],F_pos[5] };//法兰下的坐标，角度提出来
	double Rotation_Matrix[3][3] = { 0 };            //定义一个旋转矩阵
	EulerAngleToVector(angle, Rotation_Matrix);      //将欧拉角转旋转矩阵
	Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Falange_V;                  //定义一个4*4矩阵：
	Falange_V << Rotation_Matrix[0][0], Rotation_Matrix[1][0], Rotation_Matrix[2][0], F_pos[0],
		Rotation_Matrix[0][1], Rotation_Matrix[1][1], Rotation_Matrix[2][1], F_pos[1],
		Rotation_Matrix[0][2], Rotation_Matrix[1][2], Rotation_Matrix[2][2], F_pos[2],
		0, 0, 0, 1;
	/***************************************法兰下的工具坐标*********************************************************/
	//选择工具号，对应的工具数据
	getToolInfo(ToolNumber, ToolArray[ToolNumber]);
	double* ToolDate = ToolArray[ToolNumber];
	double tooldate_1[8] = { 0 };
	for (size_t i = 0; i < 6; i++)
	{
		tooldate_1[i] = ToolArray[ToolNumber][i];
	}
	double Rotation_Matrix_T[3][3] = { 0 };          //定义工具选择矩阵
	double angle_T[3] = { ToolDate[3],ToolDate[4],ToolDate[5] };
	EulerAngleToVector(angle_T, Rotation_Matrix_T);  //工具的欧拉角转旋转矩阵
	Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Tool_v;
	Tool_v << Rotation_Matrix_T[0][0], Rotation_Matrix_T[1][0], Rotation_Matrix_T[2][0], ToolDate[0],
		Rotation_Matrix_T[0][1], Rotation_Matrix_T[1][1], Rotation_Matrix_T[2][1], ToolDate[1],
		Rotation_Matrix_T[0][2], Rotation_Matrix_T[1][2], Rotation_Matrix_T[2][2], ToolDate[2],
		0, 0, 0, 1;

	/*********************************************基坐标下的用户坐标*******************************************************************/
	getUserInfo(UserNumber, UserArray[UserNumber]);
	double* UserDate = UserArray[UserNumber];
	double Userdate_1[8] = { 0 };
	for (size_t i = 0; i < 6; i++)
	{
		Userdate_1[i] = UserArray[UserNumber][i];
	}
	double Rotation_Matrix_U[3][3] = { 0 };          //定义工具选择矩阵
	double angle_U[3] = { UserDate[3],UserDate[4],UserDate[5] };
	EulerAngleToVector(angle_U, Rotation_Matrix_U);  //工具的欧拉角转旋转矩阵
	Eigen::Matrix<double, 4, 4, Eigen::RowMajor> User_v;
	User_v << Rotation_Matrix_U[0][0], Rotation_Matrix_U[1][0], Rotation_Matrix_U[2][0], UserDate[0],
		Rotation_Matrix_U[0][1], Rotation_Matrix_U[1][1], Rotation_Matrix_U[2][1], UserDate[1],
		Rotation_Matrix_U[0][2], Rotation_Matrix_U[1][2], Rotation_Matrix_U[2][2], UserDate[2],
		0, 0, 0, 1;
	/******************************************************************************************************************/

	/*******************************************计算公式***************************************************/
	Eigen::Matrix<double, 4, 4, Eigen::RowMajor> pos_FtT; //用来存 法兰矩阵*工具矩阵的结果
	pos_FtT << 0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;
	pos_FtT = User_v.inverse() * Falange_V * Tool_v; //得到用户坐标系下的工具坐标下当前位置

	//Eigen::Matrix4d pos_FtT_R = pos_FtT.transpose();
	double RotationMatrix[3][3] = { pos_FtT(0,0) , pos_FtT(0,1),pos_FtT(0,2) ,pos_FtT(1,0) , pos_FtT(1,1),pos_FtT(1,2) ,pos_FtT(2,0) , pos_FtT(2,1),pos_FtT(2,2) };
	double angle_FtT[3] = { 0 };
	RotationToEulerAngle(RotationMatrix, angle_FtT);

	T_pos[0] = pos_FtT(0, 3);
	T_pos[1] = pos_FtT(1, 3);
	T_pos[2] = pos_FtT(2, 3);
	T_pos[3] = angle_FtT[0];
	T_pos[4] = angle_FtT[1];
	T_pos[5] = angle_FtT[2];

}

void CROBOTPCtr::Tool_PosToFlange(double T_pos[6], int ToolNumber, int UserNumber, double F_pos[6])
{
	/************************************当前工具下的直角坐标**********************************************************/
	double angle[3] = { T_pos[3],T_pos[4],T_pos[5] };//法兰下的坐标，角度提出来
	double Rotation_Matrix[3][3] = { 0 };            //定义一个旋转矩阵
	EulerAngleToVector(angle, Rotation_Matrix);      //将欧拉角转旋转矩阵
	Eigen::Matrix<double, 4, 4, Eigen::RowMajor> User_T_tool;                  //定义一个4*4矩阵：
	User_T_tool << Rotation_Matrix[0][0], Rotation_Matrix[1][0], Rotation_Matrix[2][0], T_pos[0],
		Rotation_Matrix[0][1], Rotation_Matrix[1][1], Rotation_Matrix[2][1], T_pos[1],
		Rotation_Matrix[0][2], Rotation_Matrix[1][2], Rotation_Matrix[2][2], T_pos[2],
		0, 0, 0, 1;
	/***************************************法兰下的工具坐标*********************************************************/
	//选择工具号，对应的工具数据
	double* ToolDate = ToolArray[ToolNumber];
	double Rotation_Matrix_T[3][3] = { 0 };          //定义工具选择矩阵
	double angle_T[3] = { ToolDate[3],ToolDate[4],ToolDate[5] };
	EulerAngleToVector(angle_T, Rotation_Matrix_T);  //工具的欧拉角转旋转矩阵
	Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Flange_T_Tool;
	Flange_T_Tool << Rotation_Matrix_T[0][0], Rotation_Matrix_T[1][0], Rotation_Matrix_T[2][0], ToolDate[0],
		Rotation_Matrix_T[0][1], Rotation_Matrix_T[1][1], Rotation_Matrix_T[2][1], ToolDate[1],
		Rotation_Matrix_T[0][2], Rotation_Matrix_T[1][2], Rotation_Matrix_T[2][2], ToolDate[2],
		0, 0, 0, 1;

	/*********************************************基坐标下的用户坐标*******************************************************************/
	double* UserDate = UserArray[UserNumber];
	double Rotation_Matrix_U[3][3] = { 0 };          //定义工具选择矩阵
	double angle_U[3] = { UserDate[3],UserDate[4],UserDate[5] };
	EulerAngleToVector(angle_U, Rotation_Matrix_U);  //工具的欧拉角转旋转矩阵
	Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Base_T_User;
	Base_T_User << Rotation_Matrix_U[0][0], Rotation_Matrix_U[1][0], Rotation_Matrix_U[2][0], UserDate[0],
		Rotation_Matrix_U[0][1], Rotation_Matrix_U[1][1], Rotation_Matrix_U[2][1], UserDate[1],
		Rotation_Matrix_U[0][2], Rotation_Matrix_U[1][2], Rotation_Matrix_U[2][2], UserDate[2],
		0, 0, 0, 1;
	/******************************************************************************************************************/

	/*******************************************计算公式***************************************************/
	Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Base_T_Flange; //用来存 
	Base_T_Flange << 0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;
	Base_T_Flange = Base_T_User * User_T_tool * Flange_T_Tool.inverse(); //得到基坐标下的当前位置

													 //Eigen::Matrix4d pos_FtT_R = pos_FtT.transpose();
	double RotationMatrix[3][3] = { Base_T_Flange(0,0) , Base_T_Flange(0,1),Base_T_Flange(0,2) ,Base_T_Flange(1,0) , Base_T_Flange(1,1),Base_T_Flange(1,2) ,Base_T_Flange(2,0) , Base_T_Flange(2,1),Base_T_Flange(2,2) };
	double angle_FtT[3] = { 0 };
	RotationToEulerAngle(RotationMatrix, angle_FtT);

	F_pos[0] = Base_T_Flange(0, 3);
	F_pos[1] = Base_T_Flange(1, 3);
	F_pos[2] = Base_T_Flange(2, 3);
	F_pos[3] = angle_FtT[0];
	F_pos[4] = angle_FtT[1];
	F_pos[5] = angle_FtT[2];
}

//已解决sprintf问题
void CROBOTPCtr::MakeMoveL(double Pos[], double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string * moveL_str)
{
	double F_pos[8] = { 0 };
	for (size_t i = 0; i < 6; i++)
	{
		*(F_pos + i) = *(Pos + i);
	}
	//Tool_PosToFlange(Pos, Pos_Tool, Pos_User, F_pos);
	F_pos[6] = Pos[6];//外部轴7
	F_pos[7] = Pos[7];//外部轴8
	char moveL_C_a[500];
	char moveL_C_c[500];
	memset(moveL_C_c, '\0', 500);
	sprintf(moveL_C_c, " J9=0.0000000000 J10=0.0000000000 J11=0.0000000000 J12=0.0000000000 J13=0.0000000000 J14=0.0000000000 J15=0.0000000000 J16=0.0000000000");

	memset(moveL_C_a, '\0', sizeof(char) * 500);
	int E = 0;
	double angle[3] = { 0,0,0 };
	angle[0] = F_pos[3];
	angle[1] = F_pos[4];
	angle[2] = F_pos[5];
	double EulerAngle[3][3] = { 0,0,0,0,0,0,0,0,0 };
	EulerAngleToVector(angle, EulerAngle);
	if (Move_Tool > 0) E = 1;    //E=1程序是工具末端点位置
	else E = 0;                  //E=0程序是法兰位置
	sprintf(moveL_C_a, "MOVL VL=%.1f PL=%d ACC=%d DEC=%d TOOL=%d BASE=0 USE=%d COUNT=0 J1=0 J2=0 J3=0 J4=0 J5=0 J6=0 J7= ", speed, Smooth, Acc, Dec, Move_Tool, Move_UserNu);
	ostringstream oss;
	oss << moveL_C_a << F_pos[6] << " J8=" << F_pos[7] << " Nx=" << EulerAngle[0][0] << " Ny=" << EulerAngle[0][1] << " Nz=" << EulerAngle[0][2] << " Ox=" << EulerAngle[1][0] << " Oy=" << EulerAngle[1][1] << " Oz=" << EulerAngle[1][2] << " Ax=" << EulerAngle[2][0] << " Ay=" << EulerAngle[2][1] << " Az=" << EulerAngle[2][2] << " Px=" << F_pos[0] << " Py=" << F_pos[1] << " Pz=" << F_pos[2] << " E=" << E << moveL_C_c << " N=" << LineNumber;
	string a = oss.str();
	*moveL_str = a;
}

//已解决sprintf问题
void CROBOTPCtr::MakeMoveL(CRP_CPOS &CPos, double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string * MoveL_str)
{
	double F_pos[8] = { 0 };
	//Tool_PosToFlange(Pos, Pos_Tool, Pos_User, F_pos);
	for (size_t i = 0; i < 8; i++)
	{
		*(F_pos + i) = *(&CPos.X + i);
	}
	char moveL_C_a[500];
	char moveL_C_c[500];
	memset(moveL_C_c, '\0', 500);
	sprintf(moveL_C_c, " J9=0.0000000000 J10=0.0000000000 J11=0.0000000000 J12=0.0000000000 J13=0.0000000000 J14=0.0000000000 J15=0.0000000000 J16=0.0000000000");

	memset(moveL_C_a, '\0', sizeof(char) * 500);
	int E = 0;
	double angle[3] = { 0,0,0 };
	angle[0] = F_pos[3];
	angle[1] = F_pos[4];
	angle[2] = F_pos[5];
	double EulerAngle[3][3] = { 0,0,0,0,0,0,0,0,0 };
	EulerAngleToVector(angle, EulerAngle);
	if (Move_Tool > 0) E = 1;    //E=1程序是工具末端点位置
	else E = 0;                  //E=0程序是法兰位置
	sprintf(moveL_C_a, "MOVL VL=%.1f PL=%d ACC=%d DEC=%d TOOL=%d BASE=0 USE=%d COUNT=0 J1=0 J2=0 J3=0 J4=0 J5=0 J6=0 J7= ", speed, Smooth, Acc, Dec, Move_Tool, Move_UserNu);
	ostringstream oss;
	oss << moveL_C_a << F_pos[6] << " J8=" << F_pos[7] << " Nx=" << EulerAngle[0][0] << " Ny=" << EulerAngle[0][1] << " Nz=" << EulerAngle[0][2] << " Ox=" << EulerAngle[1][0] << " Oy=" << EulerAngle[1][1] << " Oz=" << EulerAngle[1][2] << " Ax=" << EulerAngle[2][0] << " Ay=" << EulerAngle[2][1] << " Az=" << EulerAngle[2][2] << " Px=" << F_pos[0] << " Py=" << F_pos[1] << " Pz=" << F_pos[2] << " E=" << E << moveL_C_c << " N=" << LineNumber;
	string a = oss.str();
	*MoveL_str = a;

}

//已解决sprintf问题
void CROBOTPCtr::MakeMoveJ(double Pos[], double speed, int Tool, int UserNu, int Acc, int Dec, int Smooth, int LineNumber, string * MoveJ_str)
{
	int E = 0;
	if (Tool > 0) E = 1;    //E=1程序是工具末端点位置
	else E = 0;            //E=0程序是法兰位置
	char moveL_C_a[500];
	char moveL_C_c[500];
	memset(moveL_C_a, '\0', 500);
	memset(moveL_C_c, '\0', 500);
	if (Tool > 0) E = 1;    //E=1程序是工具末端点位置
	else E = 0;            //E=0程序是法兰位置
	sprintf(moveL_C_a, "MOVJ VJ=%.1f PL=%d ACC=%d DEC=%d TOOL=%d BASE=0 USE=%d COUNT=0", speed, Smooth, Acc, Dec, Tool, UserNu);
	sprintf(moveL_C_c, " Nx=0 Ny=0 Nz=0 Ox=0 Oy=0 Oz=0 Ax=0 Ay=0 Az=0 Px=0 Py=0 Pz=0 E=%d  J9=0.0000000000 J10=0.0000000000 J11=0.0000000000 J12=0.0000000000 J13=0.0000000000 J14=0.0000000000 J15=0.0000000000 J16=0.0000000000 N%d", E, LineNumber);
	ostringstream oss;
	oss << moveL_C_a << " J1=" << Pos[0] << " J2=" << Pos[1] << " J3=" << Pos[2] << " J4=" << Pos[3] << " J5=" << Pos[4] << " J6=" << Pos[5] << " J7=" << Pos[6] << " J8=" << Pos[7] << moveL_C_c;
	string a = oss.str();
	*MoveJ_str = a;

	
}

//已解决sprintf问题
void CROBOTPCtr::MakeMoveJ(CRP_CPOS &CPos, double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string * MoveJ_str)
{
	double F_pos[8] = { 0 };
	for (size_t i = 0; i < 8; i++)
	{
		*(F_pos + i) = *(&CPos.X + i);
	}
	int E = 0;
	if (Move_Tool > 0) E = 1;    //E=1程序是工具末端点位置
	else E = 0;            //E=0程序是法兰位置
	char moveL_C_a[500];
	char moveL_C_c[500];
	memset(moveL_C_a, '\0', 500);
	memset(moveL_C_c, '\0', 500);
	if (Move_Tool > 0) E = 1;    //E=1程序是工具末端点位置
	else E = 0;            //E=0程序是法兰位置
	sprintf(moveL_C_a, "MOVJ VJ=%.1f PL=%d ACC=%d DEC=%d TOOL=%d BASE=0 USE=%d COUNT=0", speed, Smooth, Acc, Dec, Move_Tool, Move_UserNu);
	sprintf(moveL_C_c, " Nx=0 Ny=0 Nz=0 Ox=0 Oy=0 Oz=0 Ax=0 Ay=0 Az=0 Px=0 Py=0 Pz=0 E=%d  J9=0.0000000000 J10=0.0000000000 J11=0.0000000000 J12=0.0000000000 J13=0.0000000000 J14=0.0000000000 J15=0.0000000000 J16=0.0000000000 N%d", E, LineNumber);
	ostringstream oss;
	oss << moveL_C_a << " J1=" << F_pos[0] << " J2=" << F_pos[1] << " J3=" << F_pos[2] << " J4=" << F_pos[3] << " J5=" << F_pos[4] << " J6=" << F_pos[5] << " J7=" << F_pos[6] << " J8=" << F_pos[7] << moveL_C_c;
	string a = oss.str();
	*MoveJ_str = a;
}

//已解决sprintf问题
void CROBOTPCtr::MakeMoveJ_Axis(CRP_APOS &Axis, double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string * MoveJ_str)
{
	int E = 0;
	char moveL_C_a[500];
	char moveL_C_c[500];
	memset(moveL_C_a, '\0', 500);
	memset(moveL_C_c, '\0', 500);
	//if (Move_Tool > 0) E = 1;    //E=1程序是工具末端点位置
	//else E = 0;            //E=0程序是法兰位置
	double base_pos[10] = { 0.0 };
	if(0 != Read_GP_Number_status(95, base_pos)) {
		return;
	}
	sprintf(moveL_C_a, "MOVJ VJ=%.1f PL=%d ACC=%d DEC=%d TOOL=%d BASE=0 USE=%d COUNT=0 ", speed, Smooth, Acc, Dec, Move_Tool, Move_UserNu);
	sprintf(moveL_C_c, "Nx=0 Ny=0 Nz=0 Ox=0 Oy=0 Oz=0 Ax=0 Ay=0 Az=0 Px=%.5f Py=%.5f Pz=%.5f E=%d  J9=0.0000000000 J10=0.0000000000 J11=0.0000000000 J12=0.0000000000 J13=0.0000000000 J14=0.0000000000 J15=0.0000000000 J16=0.0000000000 N%d",base_pos[0], base_pos[1], base_pos[2], E, LineNumber);
	ostringstream oss;
	oss << moveL_C_a << "J1=" << Axis.A1 << " J2=" << Axis.A2 << " J3=" << Axis.A3 << " J4=" << Axis.A4 << " J5=" << Axis.A5 << " J6=" << Axis.A6 << " J7=" << Axis.A7 << " J8=" << Axis.A8 << " " << moveL_C_c;
	string a = oss.str();
	*MoveJ_str = a;

}

/*直角绝对运动*/
int CROBOTPCtr::MoveByJob_L(double Pos[], double speed, int Tool, int UserNu, int Acc, int Dec, bool ifSubPro)
{
	//keepAlive();
	string pro;
	string changeTool_str;
	string changeUser_str;
	string end;
	string mStrat, mEnd;
	MakeMout(500, true, 1, &mStrat);
	MakeMout(500, false, 5, &mEnd);
	MakeMoveL(Pos, speed, Tool, UserNu, Acc, Dec, 0, 4, &pro);
	if (ifSubPro)
	{
		programerEnd(6, &end);
	}
	std::ofstream RecordFile("Move_L"); //用记事本的方式记录下来
	MakeChangeTool(Tool, 2, changeTool_str);
	MakeChangeUse(UserNu, 3, changeUser_str);

	RecordFile << mStrat << "\n";
	RecordFile << changeTool_str << "\n";
	RecordFile << changeUser_str << "\n";
	RecordFile << pro << "\n";
	RecordFile << mEnd << "\n";
	RecordFile << end << "\n";
	RecordFile.close();

	if (!Obj_FTP_cmd->FTP_if_load) return -1;
	bool result = Obj_FTP_cmd->FTP_SendFile("Move_L");
	if (!result) return -1;
	return 0;
}

int CROBOTPCtr::MoveByJob_J(double Pos[], double speed, int Tool, int UserNu, int Acc, int Dec, bool ifSubPro)
{
	//keepAlive();
	string pro;
	string changeTool_str;
	string changeUser_str;
	string end;
	string mStrat, mEnd;
	MakeMout(500, true, 1, &mStrat);
	MakeMout(500, false, 5, &mEnd);
	MakeMoveJ(Pos, speed, Tool, UserNu, Acc, Dec, 0, 4, &pro);
	if (ifSubPro)
	{
		programerEnd(6, &end);
	}
	std::ofstream RecordFile("Move_J"); //用记事本的方式记录下来
	MakeChangeTool(Tool, 2, changeTool_str);
	MakeChangeUse(UserNu, 3, changeUser_str);

	RecordFile << mStrat << "\n";
	RecordFile << changeTool_str << "\n";
	RecordFile << changeUser_str << "\n";
	RecordFile << pro << "\n";
	RecordFile << mEnd << "\n";
	RecordFile << end << "\n";
	RecordFile.close();

	if (!Obj_FTP_cmd->FTP_if_load) return -1;
	bool result = Obj_FTP_cmd->FTP_SendFile("Move_J");
	if (!result) return -1;
	return 0;
}

int CROBOTPCtr::MoveByJob_J_Axis(double Axis[], double speed, int Tool, int UserNu, int Acc, int Dec, bool ifSubPro)
{
	//keepAlive();
	string pro;
	string changeTool_str;
	string changeUser_str;
	string end;
	string mStrat, mEnd;
	MakeMout(500, true, 1, &mStrat);
	MakeMout(500, false, 5, &mEnd);
	MakeMoveJ_Axis(Axis, speed, Tool, UserNu, Acc, Dec, 0, 3, &pro);
	if (ifSubPro)
	{
		programerEnd(6, &end);
	}
	std::ofstream RecordFile("Move_J_A"); //用记事本的方式记录下来
	MakeChangeTool(Tool, 2, changeTool_str);
	MakeChangeUse(UserNu, 3, changeUser_str);

	RecordFile << mStrat << "\n";
	RecordFile << changeTool_str << "\n";
	RecordFile << changeUser_str << "\n";
	RecordFile << pro << "\n";
	RecordFile << mEnd << "\n";
	RecordFile << end << "\n";

	RecordFile.close();

	if (!Obj_FTP_cmd->FTP_if_load) return -1;
	bool result = Obj_FTP_cmd->FTP_SendFile("Move_J_A");
	if (!result) return -1;
	return 0;
}

/*直角相对运动*/
int CROBOTPCtr::MoveL_Relative(double Relative[], double speed, int Tool, int UserNu, int Acc, int Dec)
{
	double pos[10] = { 0 };
	int result = GetCurrPos(pos);
	if (result < 0) return -1;
	for (size_t i = 0; i < 8; i++)
	{
		pos[i] = pos[i] + Relative[i];
	}
	return MoveByJob_L(pos, speed, Tool, UserNu, Acc, Dec);
}

int CROBOTPCtr::MoveJ_Relative(double PosRelative[], double speed, int Tool, int UserNu, int Acc, int Dec)
{
	double pos[10] = { 0 };
	int result = GetCurrAxisByAngle(pos);
	if (result < 0) return -1;
	for (size_t i = 0; i < 8; i++)
	{
		pos[i] = pos[i] + PosRelative[i];
	}
	return MoveByJob_J(pos, speed, Tool, UserNu, Acc, Dec);
}

int CROBOTPCtr::MoveJ_Relative_Axis(double Relative[], double speed, int Tool, int UserNu, int Acc, int Dec)
{
	double Axis[10] = { 0 };
	int result = GetCurrAxisByAngle(Axis);
	if (result < 0) return -1;
	for (size_t i = 0; i < 8; i++)
	{
		Axis[i] = Axis[i] + Relative[i];
	}
	return MoveByJob_J_Axis(Axis, speed, Tool, UserNu, Acc, Dec);
}

int CROBOTPCtr::Single_MoveL(int whichAxis, double distence, int ifRelative, double speed, int Tool, int UserNu, int Acc, int Dec)
{
	double pos[10] = { 0 };
	int result = GetCurrPos(pos);
	if (result < 0)return -1;
	if (ifRelative == 0)//绝对运动
	{
		pos[whichAxis] = distence;
		result = MoveByJob_L(pos, speed, Tool, UserNu, Acc, Dec);
		if (result < 0)return -1;
	}
	else //相对运动
	{
		pos[whichAxis] = pos[whichAxis] + distence;
		result = MoveByJob_L(pos, speed, Tool, UserNu, Acc, Dec);
		if (result < 0)return -1;
	}
	return 0;
}

int CROBOTPCtr::Single_MoveJ(int whichAxis, double distence, int ifRelative, double speed, int Tool, int UserNu, int Acc, int Dec)
{
	double pos[10] = { 0 };
	int result = GetCurrAxisByAngle(pos);
	if (result < 0)return -1;
	if (ifRelative == 0)//绝对运动
	{
		pos[whichAxis] = distence;
		result = MoveByJob_J(pos, speed, Tool, UserNu, Acc, Dec);
		if (result < 0)return -1;
	}
	else  //相对运动
	{
		pos[whichAxis] = pos[whichAxis] + distence;
		result = MoveByJob_J(pos, speed, Tool, UserNu, Acc, Dec);
		if (result < 0)return -1;
	}
	return 0;
}

int CROBOTPCtr::Single_MoveJ_Axis(int whichAxis, double distence, int ifRelative, double speed, int Tool, int UserNu, int Acc, int Dec)
{
	double pos[10] = { 0 };
	int result = GetCurrAxisByAngle(pos);
	if (result < 0)return -1;
	if (ifRelative == 0)//绝对运动
	{
		pos[whichAxis] = distence;
		result = MoveByJob_J_Axis(pos, speed, Tool, UserNu, Acc, Dec);
		if (result < 0)return -1;
	}
	else//相对运动
	{
		pos[whichAxis] = pos[whichAxis] + distence;
		result = MoveByJob_J_Axis(pos, speed, Tool, UserNu, Acc, Dec);
		if (result < 0)return -1;
	}
	return 0;
}

//已解决sprintf问题
void CROBOTPCtr::MakeMoveJ_Axis(double Axis[], double speed, int Tool, int UserNu, int Acc, int Dec, int Smooth, int LineNumber, string * MoveJ_str)
{
	int E = 0;
	char moveL_C_a[500];
	char moveL_C_c[500];
	memset(moveL_C_a, '\0', 500);
	memset(moveL_C_c, '\0', 500);
	if (Tool > 0) E = 1;    //E=1程序是工具末端点位置
	else E = 0;            //E=0程序是法兰位置
	sprintf(moveL_C_a, "MOVJ VJ=%.1f PL=%d ACC=%d DEC=%d TOOL=%d BASE=0 USE=%d COUNT=0", speed, Smooth, Acc, Dec, Tool, UserNu);
	sprintf(moveL_C_c, " Nx=0 Ny=0 Nz=0 Ox=0 Oy=0 Oz=0 Ax=0 Ay=0 Az=0 Px=0 Py=0 Pz=0 E=%d  J9=0.0000000000 J10=0.0000000000 J11=0.0000000000 J12=0.0000000000 J13=0.0000000000 J14=0.0000000000 J15=0.0000000000 J16=0.0000000000 N%d", E, LineNumber);
	ostringstream oss;
	oss << moveL_C_a << " J1=" << Axis[0] << " J2=" << Axis[1] << " J3=" << Axis[2] << " J4=" << Axis[3] << " J5=" << Axis[4] << " J6=" << Axis[5] << " J7=" << Axis[6] << " J8=" << Axis[7] << moveL_C_c;
	string a = oss.str();
	*MoveJ_str = a;
}



void CROBOTPCtr::MakeChangeTool(int ToolNumber, int LineNumber, string& ChangeTool_Str)
{
	char str[20];
	memset(str, '\0', sizeof(char) * 12);
	sprintf(str, "CHANGETOOL#(%d) N%d ", ToolNumber, LineNumber);
	ChangeTool_Str = str;
}

void CROBOTPCtr::MakeChangeUse(int UserNumber, int LineNumber, string & ChangeUser_Str)
{
	char str[20];
	memset(str, '\0', sizeof(char) * 12);
	sprintf(str, "CHANGEUSE#(%d) N%d ", UserNumber, LineNumber);
	ChangeUser_Str = str;
}

int CROBOTPCtr::programerEnd(int number_Line, string* EndLine)
{
	char end[12];
	memset(end, '\0', sizeof(char) * 12);
	sprintf(end, "RET N%d", number_Line);
	*EndLine = end;
	return 0;
}

int CROBOTPCtr::Move_MultiPos(double ** Pos, int Pos_Number, double speed[], int Tool, int UserNu, int LineNumber, bool ifSubPro, string FileName)
{
	string pro;
	string ChangeTool_str;
	string ChangeUser_str;
	string end;
	string mStrat, mEnd;
	std::ofstream RecordFile;
	if (LineNumber > 1)
	{
		std::ofstream RecordFile(FileName, ios::app); //用记事本的方式记录下来
		MakeMout(500, true, LineNumber, &mStrat);
		MakeChangeTool(Tool, LineNumber + 1, ChangeTool_str);
		MakeChangeUse(UserNu, LineNumber + 2, ChangeUser_str);
		RecordFile << mStrat << "\n";
		RecordFile << ChangeTool_str << "\n";
		RecordFile << ChangeUser_str << "\n";
		for (size_t i = 0; i < Pos_Number; i++)
		{
			MakeMoveL(Pos[i], speed[i], Tool, UserNu, 0, 0, 9, i + 3 + LineNumber, &pro);
			RecordFile << pro << "\n";
		}
		if (ifSubPro)
		{
			programerEnd(Pos_Number + 1, &end);
			MakeMout(500, false, Pos_Number + 2, &mEnd);
			RecordFile << end << "\n";
		}
		else {
			MakeMout(500, false, Pos_Number + 1, &mEnd);
		}
		RecordFile << mEnd << "\n";
		RecordFile.close();
		if (!Obj_FTP_cmd->FTP_if_load) return -1;
		bool result = Obj_FTP_cmd->FTP_SendFile(FileName.c_str());
		if (!result) return -1;
		return 0;
	}
	else if (LineNumber == 1)
	{
		std::ofstream RecordFile(FileName); //用记事本的方式记录下来
		MakeChangeTool(Tool, 1, ChangeTool_str);
		MakeChangeUse(UserNu, 2, ChangeUser_str);
		RecordFile << ChangeTool_str << "\n";
		RecordFile << ChangeUser_str << "\n";
		for (size_t i = 0; i < Pos_Number; i++)
		{
			MakeMoveL(Pos[i], speed[i], Tool, UserNu, 0, 0, 9, i + 3, &pro);
			RecordFile << pro << "\n";
		}
		if (ifSubPro)
		{
			programerEnd(Pos_Number + 1, &end);
		}
		RecordFile << end << "\n";
		RecordFile.close();
		if (!Obj_FTP_cmd->FTP_if_load) return -1;
		bool result = Obj_FTP_cmd->FTP_SendFile(FileName.c_str());
		if (!result) return -1;
		return 0;
	}
	return -1;
}
int CROBOTPCtr::Move_MultiPos(CRP_CPOS Pos[], int Pos_Number, double speed[], int Tool, int UserNu, int LineNumber, bool ifSubPro, string FileName)
{
	string pro;
	string ChangeTool_str;
	string ChangeUser_str;
	string end;
	string mStrat, mEnd;
	std::ofstream RecordFile;
	if (LineNumber > 1)
	{

		std::ofstream RecordFile(FileName, ios::app); //用记事本的方式记录下来
		MakeMout(500, true, LineNumber, &mStrat);
		MakeChangeTool(Tool, LineNumber + 1, ChangeTool_str);
		MakeChangeUse(UserNu, LineNumber + 2, ChangeUser_str);

		RecordFile << mStrat << "\n";
		RecordFile << ChangeTool_str << "\n";
		RecordFile << ChangeUser_str << "\n";
		for (size_t i = 0; i < Pos_Number; i++)
		{
			MakeMoveL(Pos[i], speed[i], Tool, UserNu, 0, 0, 9, i + 4 + LineNumber, &pro);
			RecordFile << pro << "\n";
		}
		if (ifSubPro)
		{
			programerEnd(Pos_Number + 1, &end);
			MakeMout(500, false, Pos_Number + 2, &mEnd);
			RecordFile << end << "\n";
			RecordFile << mEnd << "\n";
		}
		else {
			MakeMout(500, false, Pos_Number + 1, &mEnd);
		}
		RecordFile << mEnd << "\n";
		RecordFile.close();
		if (!Obj_FTP_cmd->FTP_if_load) return -1;
		bool result = Obj_FTP_cmd->FTP_SendFile(FileName.c_str());
		if (!result) return -1;
		return 0;
	}
	else if (LineNumber == 1)
	{
		std::ofstream RecordFile(FileName); //用记事本的方式记录下来

		MakeMout(500, true, 1, &mStrat);
		MakeChangeTool(Tool, 2, ChangeTool_str);
		MakeChangeUse(UserNu, 3, ChangeUser_str);
		RecordFile << mStrat << "\n";
		RecordFile << ChangeTool_str << "\n";
		RecordFile << ChangeUser_str << "\n";
		for (size_t i = 0; i < Pos_Number; i++)
		{
			MakeMoveL(Pos[i], speed[i], Tool, UserNu, 0, 0, 9, i + 3, &pro);
			RecordFile << pro << "\n";
		}
		if (ifSubPro)
		{
			MakeMout(500, false, Pos_Number + 2, &mStrat);
			programerEnd(Pos_Number + 1, &end);
		}

		RecordFile << end << "\n";
		RecordFile.close();
		if (!Obj_FTP_cmd->FTP_if_load) return -1;
		bool result = Obj_FTP_cmd->FTP_SendFile(FileName.c_str());
		if (!result) return -1;
		return 0;
	}
	return -1;
}

int CROBOTPCtr::ChangeUserNum(int newUserNum)
{
	//keepAlive();
	string ChangeUser_Str, EndLine;
	MakeChangeUse(newUserNum, 1, ChangeUser_Str);
	programerEnd(2, &EndLine);
	std::ofstream RecordFile("ChangeUser"); //用记事本的方式记录下来

	RecordFile << ChangeUser_Str << "\n";
	//snprintf(movL, 220, "MOVL VL=100.0 GP#95 PL=0 ACC=0.0 DEC=0 TOOL=%d BASE=0 USE=0 COUNT=0 J1=0 J2=0 J3=0 J4=0 J5=0 J6=0 J7=0 J8=0 Nx=0 Ny=0 Nz=0 Ox=0 Oy=0 Oz=0 Ax=0 Ay=0 Az=0 Px=0 Py=0 Pz=0 E=0 J9=0 J10=0 J11=0 J12=0 J13=0 J14=0 J15=0 J16=0 N2", newUserNum);
	RecordFile << EndLine << "\n";
	RecordFile.close();
	if (!Obj_FTP_cmd->FTP_if_load) return -1;
	bool result = Obj_FTP_cmd->FTP_SendFile("ChangeUser");
	return 0;
}

int CROBOTPCtr::ChangeToolNum(int newToolNum)
{
	//keepAlive();
	string changeTool_str;
	//string changeUser_str;
	string end;
	MakeChangeTool(newToolNum, 1, changeTool_str);
	//MakeChangeUse(UserNu, 2, changeUser_str);
	programerEnd(2, &end);

	std::ofstream RecordFile("ChangeTool"); //用记事本的方式记录下来
	RecordFile << changeTool_str << "\n";
	RecordFile << end << "\n";
	RecordFile.close();

	if (!Obj_FTP_cmd->FTP_if_load) return -1;
	bool result = Obj_FTP_cmd->FTP_SendFile("ChangeTool");
	if (!result) return -1;

	int resCall = XiRobot_CallJob(4);
	return resCall;
}

int CROBOTPCtr::CallJob(UINT ProgamerNumber)
{
	//keepAlive();
	if (ProgamerNumber < 0 || ProgamerNumber>10) return -1;
	int number = 500 + ProgamerNumber;
	if (Obj_Modbus->m_connected) {
		while (Read_M_Number_status(360) != 1)
		{
			Write_M_Number_status(number, false);
			int result = Write_M_Number_status(number, true);
			if (result < 0)return -1;
			Sleep(500);
			Write_M_Number_status(number, false);
		}
		return 0;
	}
	return -1;
}

void CROBOTPCtr::checkDone(int nDelayTime,bool showLog)
{
	//keepAlive();
	if (showLog)
	{
		FILE *p;
		p = fopen(".\\file\\CRobotM_statue.txt", "a");
		fseek(p, 0, SEEK_END);
		size_t size = ftell(p);
		if (size > 1048576 * 24) {
			fclose(p);
			remove(".\\file\\CRobotM_statue.txt");
		};

		ofstream fp(".\\file\\CRobotM_statue.txt", ios::app);
		ostringstream oss;

		oss << "****** 开始checkDone时间:" << XI_clock() << " *******" << endl;
		fp << oss.str();
		oss.str("");

		oss << "时间 " << XI_clock() << " 进入checkdone:  " << "M500= " << Read_M_Number_status(500) << "  " << /*"M21= " << Read_M_Number_status(21) << "  " <<*/ "M360= " << Read_M_Number_status(360) << endl;
		fp << oss.str();
		oss.str("");

		int nMinDelayTime = 1000; // 卡诺普相对与安川延时时间普遍较大
		nDelayTime = nDelayTime < nMinDelayTime ? nMinDelayTime : nDelayTime;

		//获取当前时间
		long long nInitialTime = XI_clock();
		//定义M500状态
		int nStopStatus = 0;

		while (nStopStatus == Read_M_Number_status(500) &&
			nStopStatus == Read_M_Number_status(360)) {
			//定义该函数的运行时间
			long long ntime = XI_clock() - nInitialTime;

			if (ntime > nDelayTime) {
				oss << "时间 " << XI_clock() << " ** 程序未在进入checkdone的" << nDelayTime << "毫秒内调用!:  " << "M500= " << Read_M_Number_status(500) << "  " /*<< "M21= " << Read_M_Number_status(21) << "  " */ << "M360= " << Read_M_Number_status(360) << endl;
				fp << oss.str();
				oss.str("");
				oss << "****** 结束checkDone时间:" << XI_clock() << " *******" << endl;
				fp << oss.str();
				fp.close();
				return;
			}
			oss << "时间 " << XI_clock() << " 等待程序调用中:  " << "M500= " << Read_M_Number_status(500) << "  "/* << "M21= " << Read_M_Number_status(21) << "  " */ << "M360= " << Read_M_Number_status(360) << endl;
			fp << oss.str();
			oss.str("");
			Sleep(100);
		}
		oss << "时间 " << XI_clock() << " 开始阻塞运行:  " << "M500= " << Read_M_Number_status(500) << "  "/* << "M21= " << Read_M_Number_status(21) << "  " */ << "M360= " << Read_M_Number_status(360) << endl;
		fp << oss.str();
		oss.str("");


		while (1) {
			if (Read_M_Number_status(500) == 0 &&
				/*Read_M_Number_status(21) == 0 &&*/
				Read_M_Number_status(360) == 0) {
				oss << "时间 " << XI_clock() << " 退出阻塞状态:  " << "M500= " << Read_M_Number_status(500) << "  " << "M21= " << Read_M_Number_status(21) << "  " << "M360= " << Read_M_Number_status(360) << endl;
				fp << oss.str();
				oss.str("");
				break;
			}
			else {
				oss << "时间 " << XI_clock() << " checkDone阻塞中:  " << "M500= " << Read_M_Number_status(500) << "  "/* << "M21= " << Read_M_Number_status(21) << "  " */ << "M360= " << Read_M_Number_status(360) << endl;
				fp << oss.str();
				oss.str("");
				Sleep(100);
			}
		}

		oss << "时间 " << XI_clock() << " checkDone阻塞已结束:  " << "M500= " << Read_M_Number_status(500) << "  "/* << "M21= " << Read_M_Number_status(21) << "  " */ << "M360= " << Read_M_Number_status(360) << endl;
		fp << oss.str();
		oss.str("");

		oss << "****** 结束checkDone时间:" << XI_clock() << " *******" << endl;
		fp << oss.str();
		oss.str("");
		fp.close();
	}
	else {
		
		int nMinDelayTime = 1000; // 卡诺普相对与安川延时时间普遍较大
		nDelayTime = nDelayTime < nMinDelayTime ? nMinDelayTime : nDelayTime;

		//获取当前时间
		long long nInitialTime = XI_clock();
		//定义M500状态
		int nStopStatus = 0;

		while (nStopStatus == Read_M_Number_status(500) &&
			nStopStatus == Read_M_Number_status(21) &&
			nStopStatus == Read_M_Number_status(360)) {
			//定义该函数的运行时间
			long long ntime = XI_clock() - nInitialTime;

			if (ntime > nDelayTime) {
				return;
			}

			Sleep(100);
		}

		while (1) {
			if (Read_M_Number_status(500) == 0 &&
				Read_M_Number_status(21) == 0 &&
				Read_M_Number_status(360) == 0) {

				break;
			}
			else {
				Sleep(100);
			}
		}
	}
	
}

void CROBOTPCtr::checkDoneByPos(int nDelayTime, bool isSingleMov)
{
	int nMinDelayTime = 1000; // 卡诺普相对与安川延时时间普遍较大
	nDelayTime = nDelayTime < nMinDelayTime ? nMinDelayTime : nDelayTime;

	int M360 = Read_M_Number_status(360);
	//获取当前时间
	long long nInitialTime = XI_clock();
	//定义M500状态
	int nStopStatus = 0;

	while (nStopStatus >= M360) {
		//定义该函数的运行时间
		long long ntime = XI_clock() - nInitialTime;

		if (ntime > nDelayTime) {
			return;
		}
		Sleep(100);
		M360 = Read_M_Number_status(360);
	}

	double pos1[10] = { 0 };
	double pos2[10] = { 0 };
	int cur = 1;
	int result = GetCurrPos(pos1);
	while (isEqual_Pos(pos1,pos2)) {
		if (isSingleMov) {
			Sleep(10);
			if (cur == 1) {
				result = GetCurrPos(pos2);
				cur = 2;
			}
			else if (cur == 2) {
				result = GetCurrPos(pos1);
				cur = 1;
			}
		}
		else {
			Sleep(300);
			if (cur == 1) {
				result = GetCurrPos(pos2);
				cur = 2;
			}
			else if (cur == 2) {
				result = GetCurrPos(pos1);
				cur = 1;
			}
		}		
	}


}

bool CROBOTPCtr::isEqual_double(double a, double b)
{
	if (a > b) {
		if (a - b <= 0.01){ return true; }
		else { return false; }
	}
	else {
		if (b - a <= 0.01) { return true; }
		else { return false; }
	}
	return false;
}

bool CROBOTPCtr::isEqual_Pos(CRP_APOS a, CRP_APOS b)
{
	if(isEqual_double(a.A1, b.A1) && 
	   isEqual_double(a.A2, b.A2) &&
	   isEqual_double(a.A3, b.A3) &&
	   isEqual_double(a.A4, b.A4) &&
	   isEqual_double(a.A5, b.A5) &&
	   isEqual_double(a.A6, b.A6) &&
	   isEqual_double(a.A7, b.A7) &&
       isEqual_double(a.A8, b.A8)    ) {
		return true;
	}
	return false;
}

bool CROBOTPCtr::isEqual_Pos(CRP_CPOS a, CRP_CPOS b)
{
	if (isEqual_double(a.X, b.X) &&
		isEqual_double(a.Y, b.Y) &&
		isEqual_double(a.Z, b.Z) &&
		isEqual_double(a.A, b.A) &&
		isEqual_double(a.B, b.B) &&
		isEqual_double(a.C, b.C) &&
		isEqual_double(a.A_7, b.A_7) &&
		isEqual_double(a.A_8, b.A_8)) {
		return true;
	}
	return false;
}

bool CROBOTPCtr::isEqual_Pos(double a[10], double b[10])
{
	for (int i = 0; i < 10; i++) {
		if (!isEqual_double(a[i], b[i])) {
			return false;
		}
	}
	return true;
}

int CROBOTPCtr::IsRunning()
{
	//keepAlive();
	//int a = Read_M_Number_status(500);  //行健自定义的运动状态检测寄存器
	//int b = Read_M_Number_status(21);
	int a = Read_M_Number_status(360);
	while (a == -1)
	{
		Sleep(10);
		a = Read_M_Number_status(360);
	}
	return a;
}

int CROBOTPCtr::CleanAlarm()
{
	//keepAlive();
	if (Obj_Modbus->m_connected) {
		int number = 512; //M512
		int result = Write_M_Number_status(number, true);
		if (result < 0)return -1;
		Sleep(200);
		result = Write_M_Number_status(number, false);
		if (result < 0)return -1;
		Sleep(200);

		result = Write_M_Number_status(number, true);
		if (result < 0)return -1;
		Sleep(200);
		result = Write_M_Number_status(number, false);
		if (result < 0)return -1;
		Sleep(200);

	}
	return 1;
}

int CROBOTPCtr::Disable()
{
	//keepAlive();
	if (Obj_FTP_cmd->m_connected/* > 0*/) {
		if (1 == Read_M_Number_status(20)) {
			int number = 511;
			int result = Write_M_Number_status(number, true);
			if (result < 0)return -1;
			Sleep(200);
			return Write_M_Number_status(number, false);
		}
		return 1;
	}
	return -1;
}

int CROBOTPCtr::Enable()
{
	//keepAlive();
	if (Obj_FTP_cmd->m_connected /*> 0 */ ) {
		if (0 == Read_M_Number_status(20)) {
			int number = 511;
			int result = Write_M_Number_status(number, true);
			if (result < 0)return -1;
			Sleep(200);
			return Write_M_Number_status(number, false);
		}
		return 1;
	}
	return -1;
}

int CROBOTPCtr::StopProgamer()
{
	//keepAlive();
	int number = 513;
	if (Obj_Modbus->m_connected) {
		int result = Write_M_Number_status(number, true);
		if (result < 0)return -1;
		Sleep(200);
		return Write_M_Number_status(number, false);
	}
	return -1;
}

int CROBOTPCtr::ResetProgamer()
{
	//keepAlive();
	if (Obj_Modbus->m_connected) {
		int number = 512; //M512
		int result = Write_M_Number_status(number, true);
		if (result < 0)return -1;
		Sleep(200);
		return Write_M_Number_status(number, false);
	}
	return -1;

}

int CROBOTPCtr::scanLaser(int state)
{
	//keepAlive();
	if (Obj_Modbus->m_connected) {
		return Write_Y_Number_status(9, state > 0);
	}
	return -1;

}

int CROBOTPCtr::trackLaser(int state)
{
	//keepAlive();
	if (Obj_Modbus->m_connected) {
		return Write_Y_Number_status(10, state > 0);
	}
	return -1;

}

int CROBOTPCtr::SetPos(int GPNumber, double date[])
{
	//keepAlive();
	if (Obj_Modbus->m_connected) {
		int reslut = Write_GP_Number_single(GPNumber, date);
		return reslut;
	}
	return -1;
}

int CROBOTPCtr::SetPos(int GPNumber, CRP_TU_CPOS & date)
{
	//keepAlive();
	if (Obj_Modbus->m_connected) {
		int reslut = Write_GP_Number_single(GPNumber, date);
		return reslut;
	}
	return -1;
}

int CROBOTPCtr::SetPos_Multi(int GPNumber, CRP_TU_CPOS date[], int count)
{
	//keepAlive();
	if (Obj_Modbus->m_connected) {
		int result = Write_GP_Number_Multi(GPNumber, date, count);
		return result;
	}
	return -1;
}

int CROBOTPCtr::GetPos(int GPNumber, double date[])
{
	//keepAlive();
	if (Obj_Modbus->m_connected) {
		int reslut = Read_GP_Number_status(GPNumber, date);
		return reslut;
	}
	return -1;
}

int CROBOTPCtr::GetCurrPos(double pos[10])
{
	//keepAlive();
	int result = 0;
	double base_pos[10] = { 0 };
	if (Obj_Modbus->m_connected)
	{
		result = Read_GP_Number_status(95, base_pos);
		//法兰下的坐标转对应工具下的直角坐标
		FlangeToTool_Pos(base_pos, (int)base_pos[8], (int)base_pos[9], pos);
		pos[6] = base_pos[6];//第7轴
		pos[7] = base_pos[7];//第8轴
		pos[8] = base_pos[8];
		pos[9] = base_pos[9];
	}
	else
	{
		result = -1;
	}
	return result;
}
int CROBOTPCtr::GetAppointToolPos(CRP_TU_CPOS& Cpos, int ToolN, int UserN)
{
	//keepAlive();
	int result = 0;
	double base_pos[10] = { 0 };
	double re_pos[10] = { 0 };
	if (Obj_Modbus->m_connected)
	{
		result = Read_GP_Number_status(95, base_pos);
		//法兰下的坐标转对应工具下的直角坐标
		FlangeToTool_Pos(base_pos, ToolN, UserN, re_pos);
		Cpos.X = re_pos[0];
		Cpos.Y = re_pos[1];
		Cpos.Z = re_pos[2];
		Cpos.A = re_pos[3];
		Cpos.B = re_pos[4];
		Cpos.C = re_pos[5];
		Cpos.A_7 = base_pos[6];//第7轴
		Cpos.A_8 = base_pos[7];//第8轴
		Cpos.Tool = ToolN;
		Cpos.User = UserN;
	}
	return result;
}
int CROBOTPCtr::GetAppointToolPos(double pos[10], int ToolN, int UserN)
{
	//keepAlive();
	int result = -1;
	double base_pos[10] = { 0 };
	if (Obj_Modbus->m_connected)
	{
		result = Read_GP_Number_status(95, base_pos);
		//法兰下的坐标转对应工具下的直角坐标
		FlangeToTool_Pos(base_pos, ToolN, UserN, pos);
		pos[6] = base_pos[6];//第7轴
		pos[7] = base_pos[7];//第8轴
		pos[8] = ToolN;  //工具
		pos[9] = UserN;  //用户
		result = 0;
	}
	return result;
}
int CROBOTPCtr::GetCurrAxisByAngle(double Axis[10])
{
	//keepAlive();
	if (Obj_Modbus->m_connected) {
		int result = Read_retain_word(0xfde8, 10, Axis);
		return result;
	}
	return -1;
}
int CROBOTPCtr::GetCurrAxis(double pulse[10])
{
	//keepAlive();
	double axis[10] = { 0.0 };
	if (Obj_Modbus->m_connected) {
		int result = Read_retain_word(0xfde8, 10, axis);
		if (result >= 0) {
			pulse[0] = 105820 + (axis[0] - 0.0007) * 40413.866;
			pulse[1] = -1262952 + (axis[1] - 89.9998) * 40345.7822;
			pulse[2] = -1316536 + (axis[2] + 0.0004) * -38223.50803;
			pulse[3] = -20027 + (axis[3] + 0.0001) * -35810.690844444;
			pulse[4] = 4144463 + (axis[4] - 89.9998) * 31554.12774;
			pulse[5] = 1542269 + (axis[5] - 0.0003) * -15254.232177777;
		}
		return result;
	}
	return -1;


}
int CROBOTPCtr::GetCurrToolNo()
{
	//keepAlive();
	double base_pos[10] = { 0 };
	if (Obj_Modbus->m_connected) {
		int result = Read_GP_Number_status(95, base_pos);
		return base_pos[8];
	}
	return -1;
}
int CROBOTPCtr::GetCurrUserNo()
{
	//keepAlive();
	double base_pos[10] = { 0 };
	if (Obj_Modbus->m_connected) {
		int result = Read_GP_Number_status(95, base_pos);
		return base_pos[9];
	}
	return -1;
}
int CROBOTPCtr::XiRobot_CallJob(int numGI)
{
	//keepAlive();
	if (Obj_Modbus->m_connected) {
		while (/*1 == Read_M_Number_status(21) ||*/ 1 == Read_M_Number_status(360)) {} //等待当前机械臂运行结束
		Sleep(250);
		int resSetGI = Write_GI_Number_status(0, numGI);
		if (resSetGI >= 0) {
			int res = CallJob(1);
			if (res >= 0)return 0;
		}
		return -1;
	}
	return -1;
}
void CROBOTPCtr::MakeDout(int indexCur, bool state, int lineNum, string * Dout_str)
{
	char Dout[50];
	memset(Dout, '\0', 50);
	if (state) {
		sprintf(Dout, "DOUT Y#(%d)=ON N%d ", indexCur, lineNum);
	}
	else {
		sprintf(Dout, "DOUT Y#(%d)=OFF N%d ", indexCur, lineNum);
	}
	*Dout_str = Dout;
}

void CROBOTPCtr::MakeMout(int indexCur, bool state, int lineNum, string * Dout_str)
{
	char Dout[50];
	memset(Dout, '\0', 50);
	if (state) {
		sprintf(Dout, "DOUT M#(%d)=ON N%d ", indexCur, lineNum);
	}
	else {
		sprintf(Dout, "DOUT M#(%d)=OFF N%d ", indexCur, lineNum);
	}
	*Dout_str = Dout;
}

void CROBOTPCtr::MakeSetGi(int index, int indexVal, int lineNum, string * SetGI_str)
{
	char SetGI[50];
	memset(SetGI, '\0', 50);
	sprintf(SetGI, "SET GI#(%d) GI#(%d) N%d", index, indexVal, lineNum);
	*SetGI_str = SetGI;
}

void CROBOTPCtr::IncGi(int index, int lineNum, string * IncGI_str)
{
	char IncGI[20];
	memset(IncGI, '\0', 20);
	sprintf(IncGI, "INC GI#(%d) N%d", index, lineNum);
	*IncGI_str = IncGI;
}

void CROBOTPCtr::loopEnd(int lineNum, string target, string * endLoop_str)
{
	char endLoop[20];
	memset(endLoop, '\0', 20);
	sprintf(endLoop, " N%d", lineNum);
	string tmp;
	tmp += "* ";
	tmp += target;
	tmp += endLoop;
	*endLoop_str = tmp;
}

void CROBOTPCtr::MakeTime(int GINum, int lineNum, string * time_str)
{
	char str[20];
	memset(str, '\0', 20);
	sprintf(str, "TIME GINo=%d N%d", GINum, lineNum);
	*time_str = str;
}

void CROBOTPCtr::MakePause(int GINum, int GPNum, int lineNum, string * pause_str)
{
	char str[50];
	memset(str, '\0', 50);
	sprintf(str, "PAUSE IF GI#(%d) == GP#(%d) N%d", GINum, GPNum, lineNum);
	*pause_str = str;
}

void CROBOTPCtr::MakePause(int YNum, int lineNum, bool state, string * pause_str)
{
	char str[50];
	memset(str, '\0', 50);
	if (state) {
		sprintf(str, "PAUSE IF Y#(%d) == ON N%d", YNum, lineNum);
	}
	else {
		sprintf(str, "PAUSE IF Y#(%d) == OFF N%d", YNum, lineNum);
	}
	*pause_str = str;
}

void CROBOTPCtr::MakeArc(int arcNum, int arcType, int SpeedType, double speed, double voltage, double current, int user, int lineNum, string* arc_str)
{
	char str[100];
	memset(str, '\0', 100);

	if (arcType == 0) {
		sprintf(str, "ARCSTART#(%d) %d %f %f %f %d N%d", arcNum, SpeedType, speed, voltage, current, user, lineNum);
	}
	else if (arcType == 1) {
		sprintf(str, "ARCSTART#(GI = %d) %d %f %f %f %d N%d", arcNum, SpeedType, speed, voltage, current, user, lineNum);
	}
	*arc_str = str;
}

void CROBOTPCtr::MakeArc(int arcNum, int arcType, int SpeedType, double speed, int user, int lineNum, string * arc_str)
{
	char str[100];
	memset(str, '\0', 100);
	if (arcType == 0) {
		sprintf(str, "ARCSTART#(%d) %d %f %d N%d", arcNum, SpeedType, speed, user, lineNum);
	}
	else if (arcType == 1) {
		sprintf(str, "ARCSTART#(GI = %d) %d %f %d N%d", arcNum, SpeedType, speed, user, lineNum);
	}
	*arc_str = str;
}

void CROBOTPCtr::MakeArcEnd(int arcNum, int lineNum, string * arcEnd_str)
{
	char str[50];
	memset(str, '\0', 50);
	sprintf(str, "ARCEND#(%d) N%d", arcNum, lineNum);
	*arcEnd_str = str;
}

void CROBOTPCtr::MakeArcEndGI(int GINum, int lineNum, string * arcEnd_str)
{
	char str[50];
	memset(str, '\0', 50);
	sprintf(str, "ARCEND#(GI = %d) N%d", GINum, lineNum);
	*arcEnd_str = str;
}

void CROBOTPCtr::MakeWaveStart(int id, int linenum, string * waveStart_str)
{
	char str[50];
	memset(str, '\0', 50);
	sprintf(str, "WEAVESINE#(0) 0 0 N%d ",linenum);
	*waveStart_str = str;
}

void CROBOTPCtr::MakeWaveEnd(int linenum, string * waveEnd_str)
{
	char str[50];
	memset(str, '\0', 50);
	sprintf(str, "WEAVEEND N%d ", linenum);
	*waveEnd_str = str;
}

void CROBOTPCtr::MakeSpot(int spotNum, int fileS, double pressure, int lineNum, string * spot_str)
{
	char str[50];
	memset(str, '\0', 50);
	sprintf(str, "SPOT#(%d) S#(%d) P#(%f) N%d", spotNum, fileS, pressure, lineNum);
	*spot_str = str;
}

void CROBOTPCtr::MakeNote(string textMsg, int lineNum, string * note_str)
{
	char str[100];
	memset(str, '\0', 100);
	sprintf(str, "; %s N%d", textMsg.c_str(), lineNum);
	*note_str = str;
}

void CROBOTPCtr::MakeSpeed(double speed, int lineNum, string * speed_str)
{
	char str[50];
	memset(str, '\0', 50);
	sprintf(str, "SPEED VL = %f N%d", speed, lineNum);
	*speed_str = str;
}

void CROBOTPCtr::MakeIF(int GINum1, int GINum2, int lineNum, string * if_str)
{
	char str[50];
	memset(str, '\0', 50);
	sprintf(str, "IF GI#(%d) == GI#(%d) 1 N%d", GINum1, GINum2, lineNum);
	*if_str = str;
}

void CROBOTPCtr::MakeELSE(int GINum, int lineNum, string * else_str)
{
	char str[50];
	memset(str, '\0', 50);
	sprintf(str, "ELSE %d N%d", GINum, lineNum);
	*else_str = str;
}

void CROBOTPCtr::MakeELSEIF(int GINum1, int GINum2, int lineNum, string * elseif_str)
{
	char str[50];
	memset(str, '\0', 50);
	sprintf(str, "ELSEIF GI#(%d) == GI#(%d) N%d", GINum1, GINum2, lineNum);
	*elseif_str = str;
}

void CROBOTPCtr::MakeWhile(int GINum1, int GINum2, int mark, int lineNum, string * while_str)
{
	char str[50];
	memset(str, '\0', 50);
	sprintf(str, "WHILE GI#(%d) == GI#(%d) %d N%d", GINum1, GINum2, mark, lineNum);
	*while_str = str;
}

void CROBOTPCtr::MakeJump(int indexCur, string target, int indexTarget, int lineNum, string * Jump_str)
{
	char Jump[100];
	memset(Jump, '\0', 100);
	sprintf(Jump, "  IF GI#(%d)==GI#(%d) N%d", indexCur, indexTarget, lineNum);
	string tmp;
	tmp += "JUMP *";
	tmp += target;
	tmp += Jump;
	*Jump_str = tmp;
}

void CROBOTPCtr::MakeMoveJByGP(int GPIndex, double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string * MoveJ_str)
{
	char moveL_C[1000];
	memset(moveL_C, '\0', 1000);
	if (GPIndex > 9) {
		sprintf(moveL_C, "MOVJ VJ=%f  GP#%d PL=%d ACC=%d DEC=%d TOOL=%d BASE=0 USE=%d COUNT=0 J1=0.0000000000 J2=0.0000000000 J3=0.0000000000 J4=0.0000000000 J5=0.0000000000 J6=0.0000000000 J7=0.0000000000 J8=0.0000000000 Nx=0.0000000000 Ny=0.0000000000 Nz=0.0000000000 Ox=0.0000000000 Oy=0.0000000000 Oz=0.0000000000 Ax=0.0000000000 Ay=0.0000000000 Az=0.0000000000 Px=0.0000000000 Py=0.0000000000 Pz=0.0000000000 E=0.00000 J9=0.0000000000 J10=0.0000000000 J11=0.0000000000 J12=0.0000000000 J13=0.0000000000 J14=0.0000000000 J15=0.0000000000 J16=0.0000000000 N%d", speed, GPIndex, Smooth, Acc, Dec, Move_Tool, Move_UserNu, LineNumber);
	}
	else {
		sprintf(moveL_C, "MOVJ VJ=%f  GP#0%d PL=%d ACC=%d DEC=%d TOOL=%d BASE=0 USE=%d COUNT=0 J1=0.0000000000 J2=0.0000000000 J3=0.0000000000 J4=0.0000000000 J5=0.0000000000 J6=0.0000000000 J7=0.0000000000 J8=0.0000000000 Nx=0.0000000000 Ny=0.0000000000 Nz=0.0000000000 Ox=0.0000000000 Oy=0.0000000000 Oz=0.0000000000 Ax=0.0000000000 Ay=0.0000000000 Az=0.0000000000 Px=0.0000000000 Py=0.0000000000 Pz=0.0000000000 E=0.00000 J9=0.0000000000 J10=0.0000000000 J11=0.0000000000 J12=0.0000000000 J13=0.0000000000 J14=0.0000000000 J15=0.0000000000 J16=0.0000000000 N%d", speed, GPIndex, Smooth, Acc, Dec, Move_Tool, Move_UserNu, LineNumber);
	}
	*MoveJ_str = moveL_C;
}

void CROBOTPCtr::MakeMoveLByGP(int GPIndex, double speed, int Move_Tool, int Move_UserNu, int Acc, int Dec, int Smooth, int LineNumber, string * MoveL_str)
{
	char moveL_C[1000];
	memset(moveL_C, '\0', 1000);
	if (GPIndex > 9) {
		sprintf(moveL_C, "MOVL VL=%f  GP#%d PL=%d ACC=%d DEC=%d TOOL=%d BASE=0 USE=%d COUNT=0 J1=0.0000000000 J2=0.0000000000 J3=0.0000000000 J4=0.0000000000 J5=0.0000000000 J6=0.0000000000 J7=0.0000000000 J8=0.0000000000 Nx=0.0000000000 Ny=0.0000000000 Nz=0.0000000000 Ox=0.0000000000 Oy=0.0000000000 Oz=0.0000000000 Ax=0.0000000000 Ay=0.0000000000 Az=0.0000000000 Px=0.0000000000 Py=0.0000000000 Pz=0.0000000000 E=0.00000 J9=0.0000000000 J10=0.0000000000 J11=0.0000000000 J12=0.0000000000 J13=0.0000000000 J14=0.0000000000 J15=0.0000000000 J16=0.0000000000 N%d", speed, GPIndex, Smooth, Acc, Dec, Move_Tool, Move_UserNu, LineNumber);
	}
	else {
		sprintf(moveL_C, "MOVL VL=%f  GP#0%d PL=%d ACC=%d DEC=%d TOOL=%d BASE=0 USE=%d COUNT=0 J1=0.0000000000 J2=0.0000000000 J3=0.0000000000 J4=0.0000000000 J5=0.0000000000 J6=0.0000000000 J7=0.0000000000 J8=0.0000000000 Nx=0.0000000000 Ny=0.0000000000 Nz=0.0000000000 Ox=0.0000000000 Oy=0.0000000000 Oz=0.0000000000 Ax=0.0000000000 Ay=0.0000000000 Az=0.0000000000 Px=0.0000000000 Py=0.0000000000 Pz=0.0000000000 E=0.00000 J9=0.0000000000 J10=0.0000000000 J11=0.0000000000 J12=0.0000000000 J13=0.0000000000 J14=0.0000000000 J15=0.0000000000 J16=0.0000000000 N%d", speed, GPIndex, Smooth, Acc, Dec, Move_Tool, Move_UserNu, LineNumber);
	}
	*MoveL_str = moveL_C;
}

bool CROBOTPCtr::SendTeachMoveData(int pointNum, double pos[][10], int pointType[], double speed, int LineScanOrTrack)
{
	if (Obj_Modbus->m_connected)
	{
		int resGP = 0, resGI = 0;
		// 将所有点位信息存在GP中，同时将点位类型存在GI中
		for (int i = 0; i < pointNum; i++)
		{
			resGP = Write_GP_Number_single(i + 10, pos[i]);
			resGI = Write_GI_Number_status(i + 10, 0); //全部初始化为0
			if (resGP < 0 || resGI < 0) return false;
		}
		string movJ, incGI, jump;
		string doutOff1, doutOn, doutOff2;
		string changeTool_str;
		string changeUser_str;
		string end, endLoop;
		std::ofstream RecordFile("TeachMove"); //创建TeachMove文件
		MakeChangeTool(1, 1, changeTool_str);
		MakeChangeUse(0, 2, changeUser_str);

		RecordFile << changeTool_str << "\n";
		RecordFile << changeUser_str << "\n";
		int lineNum = 3;
		for (int i = 0; i < pointNum; i++) {
			MakeJump(0, "endLoop", 1, lineNum++, &jump); // 第一个参数是标识（当前是第个点）第二个是指定的跳转目标，第三个是标识用来匹配的目标target
			/*使用局部变量P 优点是不会程序内不会出现类似于浅拷贝的问题，缺点是示教器上看不到具体的点
			MakeMoveJ(pos[i], speed, 1, 0, 0, 0, 0, lineNum++, &movJ); */
			/*使用全局变量P 优点是示教器上看着很清楚。 缺点是不同程序可能会出现浅拷贝问题！容易引发工伤*/
			MakeMoveJByGP(i + 10, speed, 1, 0, 0, 0, 0, lineNum++, &movJ);
			IncGi(0, lineNum++, &incGI);
			if (pointType[i] == 1) 
			{
				if (LineScanOrTrack == 1) 
				{  //Y11对应线扫，Y12对应跟踪。两个相机触发都是上升沿触发。
					MakeDout(12, true, lineNum++, &doutOff1);
					MakeDout(12, true, lineNum++, &doutOn);
					MakeDout(12, true, lineNum++, &doutOff2);
				}
				else 
				{
					MakeDout(11, true, lineNum++, &doutOff1);
					MakeDout(11, true, lineNum++, &doutOn);
					MakeDout(11, true, lineNum++, &doutOff2);
				}

			}

			RecordFile << jump << "\n";
			RecordFile << movJ << "\n";
			if (pointType[i] == 1) {
				RecordFile << doutOff1 << "\n";
				RecordFile << doutOn << "\n";
				RecordFile << doutOff2 << "\n";
			}
			RecordFile << incGI << "\n";
			if (i == pointNum - 1) {
				loopEnd(lineNum++, "endLoop", &endLoop);
				programerEnd(lineNum++, &end);
				RecordFile << endLoop << "\n";
				RecordFile << end << "\n";
			}
		}
		RecordFile.close();
		if (!Obj_FTP_cmd->FTP_if_load) return false;
		bool result = Obj_FTP_cmd->FTP_SendFile("TeachMove");
		return result;
	}
	return false;
}

int CROBOTPCtr::Write_GP_Number_single(int GPNumber, double data[])
{
	int result = 0;
	uint8_t  arr[4] = { 0,0,0,0 };
	uint8_t data_a[40];
	memset(data_a, '\0', sizeof(data_a));
	sendBuf->clear();
	sendBuf->push_back(0x3000 + GPNumber * 20);
	sendBuf->push_back(10);
	int cmd = Write_Multi_retain_register;
	for (int i = 0; i < 10; i++)
	{
		memset(arr, '\0', sizeof(arr));
		*(float*)arr = data[i];
		int k = (i + 1) * 4 - 1;
		for (int j = 0; j < 4; j++)
		{
			data_a[k] = arr[j];
			k--;
		}
	}
	for (size_t i = 0; i < 40; i++)
	{
		sendBuf->push_back(*(data_a + i));
	}
	int flat = ModbusTCP(cmd, sendBuf);
	if (wait(flat, 1000) != 0)return -1;
	map<int, vector<uint8_t>>::iterator it = Modbus_rcvBuf->find(flat);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = 2;
	result = it->second.at(4) == 10;
	ReleaseMutex(m_mutex);
	return result;
}

int CROBOTPCtr::Write_GP_Number_single(int GPNumber, CRP_TU_CPOS & date)
{
	int result = 0;
	uint8_t  arr[4]= { 0,0,0,0 };
	uint8_t data_a[40];
	memset(data_a, '\0', sizeof(data_a));
	sendBuf->clear();
	sendBuf->push_back(0x3000 + GPNumber * 20);
	sendBuf->push_back(10);
	int cmd = Write_Multi_retain_register;
	for (int i = 0; i < 10; i++)
	{
		memset(arr, '\0', sizeof(arr));
		*(float*)arr = *(&(date.X) + i);
		int k = (i + 1) * 4 - 1;
		for (int j = 0; j < 4; j++)
		{
			data_a[k] = arr[j];
			k--;
		}
	}
	for (size_t i = 0; i < 40; i++)
	{
		sendBuf->push_back(*(data_a + i));
	}
	int flat = ModbusTCP(cmd, sendBuf);
	if (wait(flat, 1000) != 0)return -1;
	map<int, vector<uint8_t>>::iterator it = Modbus_rcvBuf->find(flat);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = 2;
	result = it->second.at(4) == 10;
	ReleaseMutex(m_mutex);
	return result;
}

int CROBOTPCtr::Write_GP_Number_Multi(int GPNumber, CRP_TU_CPOS date[], int count)
{
	if (count > 63) return -1;
	int result = 0;
	uint8_t  arr[4] = { 0,0,0,0 };
	//uint8_t data_a[count*40];
	int malloc_size = sizeof(uint8_t) * 40 * count;
	uint8_t* data_a = (uint8_t*)(malloc(sizeof(uint8_t) * 40 * count));
	int size = sizeof(data_a);
	memset(data_a, '\0', malloc_size);
	sendBuf->clear();
	sendBuf->push_back(0x3000 + GPNumber * 20);
	sendBuf->push_back(count * 10);
	int cmd = Write_Multi_retain_register;
	//GP转字节，1个GP,转40字节：1*10*4；
	for (size_t j = 0; j < count; j++)
	{
		for (int i = 0; i < 10; i++)
		{
			memset(arr, '\0', sizeof(arr));
			*(float*)arr = *(&(date[j].X) + i);
			int k = (i + 1) * 4 - 1;
			for (int j = 0; j < 4; j++)
			{
				data_a[k] = arr[j];
				k--;
			}
		}
	}
	//把转好的字节塞到SendBuf里面
	for (size_t i = 0; i < count * 40; i++)
	{
		sendBuf->push_back(*(data_a + i));
	}
	int flat = ModbusTCP(cmd, sendBuf);
	//释放内存
	free(data_a);
	//赋值为空
	data_a = nullptr;

	if (wait(flat, 1000) != 0)return -1;
	map<int, vector<uint8_t>>::iterator it = Modbus_rcvBuf->find(flat);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = 2;
	result = it->second.at(4) == 10;
	ReleaseMutex(m_mutex);
	return result;
}

int CROBOTPCtr::Write_Y_Number_status(int YNumber, bool status)
{
	bool result = false;
	sendBuf->clear();
	sendBuf->push_back(YNumber);
	status ? sendBuf->push_back(1) : sendBuf->push_back(0);
	int flat = ModbusTCP(Write_coil_bite_register, sendBuf);
	if (wait(flat, 1000) != 0)return -1;
	map<int, vector<uint8_t>>::iterator it = Modbus_rcvBuf->find(flat);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = 2;
	if (status)
		result = it->second.at(11) == 0xff;
	else
		result = it->second.at(11) == 0x00;
	ReleaseMutex(m_mutex);
	return result;
}
int CROBOTPCtr::Read_M_Number_status(int MNumber)
{
	int result[3] = {0};
	int weight[2] = { 0 };
	int ans = 0;
	try
	{		
		for (int i = 0; i < sizeof(result) / sizeof(int); i++) {
			vector<uint16_t> *sendBuf1 = new vector<uint16_t>();
			if (!sendBuf1->empty()) {
				sendBuf1->clear();
			}

			sendBuf1->push_back(4096 + MNumber);
			sendBuf1->push_back(1);
			
		
			int cmd = 0;
			if (MNumber <= 95)
				cmd = Read_Input_bite_register;
			else
				cmd = Read_coil_bite_register;
			int flat = ModbusTCP(cmd, sendBuf1);
			if (wait(flat, 1000) != 0)return -1;
			map<int, vector<uint8_t>>::iterator it = Modbus_rcvBuf->find(flat);
			WaitForSingleObject(m_mutex, INFINITE);
			it->second.at(0) = 2;
			int len = it->second.at(9);
			if (len == 1)
			{
				result[i] = it->second.at(10) & 0x01;
			}
			ReleaseMutex(m_mutex);
			Sleep(10);
		}
		for (int i = 0; i < sizeof(result) / sizeof(int); i++) {
			weight[result[i]]++;
		}
		ans = weight[0] > weight[1] ? 0 : 1;
	}
	catch (...)
	{
		delete sendBuf;
		sendBuf = new vector<uint16_t>();
		Read_M_Number_status(MNumber);
	}
	//Sleep(100);
	return ans;
}

int CROBOTPCtr::FTP_land(char * name, char * password)
{
	int userRe = Obj_FTP_cmd->user(name);
	int passRe = Obj_FTP_cmd->pass(password);
	if ((userRe != 0) && (passRe != 0))
	{
		return -1;
	}
	return 0;
}

int CROBOTPCtr::Read_retain_word(int addr, int len_i, double date[])
{
	int result = 0;
	sendBuf->clear();
	sendBuf->push_back(addr);
	sendBuf->push_back(len_i * 2);
	int cmd = Read_retain_word_register;
	int flag = ModbusTCP(cmd, sendBuf);
	if (wait(flag, 1000) != 0)return -1;
	map<int, vector<uint8_t>>::iterator it = Modbus_rcvBuf->find(flag);
	WaitForSingleObject(m_mutex, 5000);
	it->second.at(0) = 2;
	int len = it->second.at(9) / 2;
	//uint16_t  data_x, data_y;
	uint8_t  arr[4] = { 0,0,0,0 };
	if (len == len_i * 2)
	{
		int number = 10;

		for (int i = 0; i < len_i; i++)
		{
			for (int j = 3; j >= 0; j--)
			{
				*(arr + j) = it->second.at(number++);
				//number++;
			}
			*(date + i) = *(float*)(arr);
		}
	}
	ReleaseMutex(m_mutex);
	return result;
}

void CROBOTPCtr::Modbus_Rcv(void * pOwner, const char * rcvData, int nDataLen)
{
	//int* a = new int[nDataLen];
	CROBOTPCtr* pThis = (CROBOTPCtr*)pOwner;
	vector<uint8_t> buf;
	//int a[20];
	buf.push_back(1);//表示没有使用过
	for (int i = 0; i < nDataLen; i++)
	{
		buf.push_back(*(rcvData + i));
	}
	if (buf.at(8) == 0x01)//01功能码
	{
		int cmd_number = buf.at(1) << 8 | buf.at(2);
		map<int, vector<uint8_t>>::iterator it = pThis->Modbus_rcvBuf->find(cmd_number);  //把int的cmd放到键值对的int里面
		WaitForSingleObject(pThis->m_mutex, 5000);                       //加锁，只允许一个线程访问
		if (it == pThis->Modbus_rcvBuf->end())                                //
			pThis->Modbus_rcvBuf->insert(make_pair(cmd_number, buf));              //把buf插入到map中
		else
			it->second = buf;
		ReleaseMutex(pThis->m_mutex);
	}
	if (buf.at(8) == 0x05)//05功能码
	{
		int cmd_number = buf.at(1) << 8 | buf.at(2);
		map<int, vector<uint8_t>>::iterator it = pThis->Modbus_rcvBuf->find(cmd_number);  //把int的cmd放到键值对的int里面
		WaitForSingleObject(pThis->m_mutex, 5000);                       //加锁，只允许一个线程访问
		if (it == pThis->Modbus_rcvBuf->end())                                //
			pThis->Modbus_rcvBuf->insert(make_pair(cmd_number, buf));              //把buf插入到map中
		else
			it->second = buf;
		ReleaseMutex(pThis->m_mutex);
	}
	if (buf.at(8) == 0x02)//02功能码
	{
		int cmd_number = buf.at(1) << 8 | buf.at(2);
		map<int, vector<uint8_t>>::iterator it = pThis->Modbus_rcvBuf->find(cmd_number);  //把int的cmd放到键值对的int里面
		WaitForSingleObject(pThis->m_mutex, 5000);                       //加锁，只允许一个线程访问
		if (it == pThis->Modbus_rcvBuf->end())                                //
			pThis->Modbus_rcvBuf->insert(make_pair(cmd_number, buf));              //把buf插入到map中
		else
			it->second = buf;
		ReleaseMutex(pThis->m_mutex);
	}
	if (buf.at(8) == 0x03)//03功能码
	{
		int cmd_number = buf.at(1) << 8 | buf.at(2);
		map<int, vector<uint8_t>>::iterator it = pThis->Modbus_rcvBuf->find(cmd_number);  //把int的cmd放到键值对的int里面
		WaitForSingleObject(pThis->m_mutex, 5000);                       //加锁，只允许一个线程访问
		if (it == pThis->Modbus_rcvBuf->end())                                //
			pThis->Modbus_rcvBuf->insert(make_pair(cmd_number, buf));              //把buf插入到map中
		else
			it->second = buf;
		/*******************************************************************************************************/

		/*******************************************************************************************/
		ReleaseMutex(pThis->m_mutex);
	}
	if (buf.at(8) == 0x06)//06功能码
	{
		int cmd_number = buf.at(1) << 8 | buf.at(2);
		map<int, vector<uint8_t>>::iterator it = pThis->Modbus_rcvBuf->find(cmd_number);  //把int的cmd放到键值对的int里面
		WaitForSingleObject(pThis->m_mutex, 5000);                       //加锁，只允许一个线程访问
		if (it == pThis->Modbus_rcvBuf->end())                                //
			pThis->Modbus_rcvBuf->insert(make_pair(cmd_number, buf));              //把buf插入到map中
		else
			it->second = buf;
		ReleaseMutex(pThis->m_mutex);
	}
	if (buf.at(8) == 0x10)//16功能码
	{
		int cmd_number = buf.at(1) << 8 | buf.at(2);
		map<int, vector<uint8_t>>::iterator it = pThis->Modbus_rcvBuf->find(cmd_number);  //把int的cmd放到键值对的int里面
		WaitForSingleObject(pThis->m_mutex, 5000);                       //加锁，只允许一个线程访问
		if (it == pThis->Modbus_rcvBuf->end())                                //
			pThis->Modbus_rcvBuf->insert(make_pair(cmd_number, buf));              //把buf插入到map中
		else
			it->second = buf;
		ReleaseMutex(pThis->m_mutex);
	}

}

void CROBOTPCtr::FTP_Rcv(void * pOwner, const char * rcvData, int nDataLen)
{
	CROBOTPCtr* pThis = (CROBOTPCtr*)pOwner;
	if (strncmp(rcvData, "220", 3) == 0)//220在ftp表示连接成功
	{
		pThis->Obj_FTP_cmd->FTP_if_connection = true;
		pThis->FTP_land(pThis->FTPname, pThis->FTPpassword);
	}
	if (strncmp(rcvData, "227", 3) == 0)//227在ftp表示被动模式
	{
		pThis->Obj_FTP_cmd->FTP_if_passive = true;
	}
	if (strncmp(rcvData, "226", 3) == 0)//226关闭数据连接
	{
		pThis->Obj_FTP_cmd->FTP_if_upComplete = true;
	}
	if ((strncmp(rcvData, "230", 3) == 0) || (strncmp(rcvData, "331", 3) == 0))//227在ftp表示登录完成
	{
		pThis->Obj_FTP_cmd->FTP_if_load = true;
	}
	if (strncmp(rcvData, "421", 3) == 0)//421服务不可用，正在关闭控制连接
	{
		pThis->Obj_FTP_cmd->FTP_if_connection = false;
		pThis->Obj_FTP_cmd->FTP_if_load = false;
		pThis->Obj_FTP_cmd->FTP_if_passive = false;
		pThis->Obj_FTP_cmd->FTP_if_upComplete = false;
	}

}

int CROBOTPCtr::ModbusTCP(int FunctionNumber, vector<uint16_t>* m_sendBuf)
{
	//char * Mod_Buff[20];
	int leng = 0;
	int len = 0;
	if (Modbus_Flag >= 0xffff)
	{
		Modbus_Flag = 0;
	}
	else
	{
		Modbus_Flag++;//事务标识符（目前响应的是第几个报文）
	}
	memset(Mod_Buff, '\0', sizeof(Mod_Buff)); //Mod_Buff初始化为'\0'
	switch (FunctionNumber)
	{
	case Read_coil_bite_register://0x01
		//Mod_Buff[0] = m_sendBuf->at(0) >> 8 ;  //传输标识符 (变量) >> 8
		//Mod_Buff[1] = m_sendBuf->at(0) &  0x00ff;  //传输标识符 (变量) &  0x00ff
		Mod_Buff[0] = Modbus_Flag >> 8;
		Mod_Buff[1] = Modbus_Flag & 0x00ff;
		Mod_Buff[2] = 0x00;  //协议标识符
		Mod_Buff[3] = 0x00;  //协议标识符
		Mod_Buff[4] = 0x00;  //字节长度
		Mod_Buff[5] = 0x06;  //字节长度
		Mod_Buff[6] = 0x01;  //站号
		Mod_Buff[7] = 0x01;  //功能码
		Mod_Buff[8] = m_sendBuf->at(0) >> 8;   //地址（高八位）         （变量）>> 8
		Mod_Buff[9] = m_sendBuf->at(0) & 0x00ff;  //地址（低八位）     （变量）&  0x00ff
		Mod_Buff[10] = m_sendBuf->at(1) >> 8;  //读几位（高八位）       (变量) >> 8
		Mod_Buff[11] = m_sendBuf->at(1) & 0x00ff; //读几位（低八位）  （变量）&  0x00ff
		len = 12;
		break;
	case Read_Input_bite_register://0x02
		Mod_Buff[0] = Modbus_Flag >> 8;
		Mod_Buff[1] = Modbus_Flag & 0x00ff;
		Mod_Buff[2] = 0x00;  //协议标识符
		Mod_Buff[3] = 0x00;  //协议标识符
		Mod_Buff[4] = 0x00;  //字节长度
		Mod_Buff[5] = 0x06;  //字节长度
		Mod_Buff[6] = 0x01;  //站号
		Mod_Buff[7] = 0x02;  //功能码
		Mod_Buff[8] = m_sendBuf->at(0) >> 8;  //地址     （变量）>> 8
		Mod_Buff[9] = m_sendBuf->at(0) & 0x00ff;  //地址     （变量）&  0x00ff
		Mod_Buff[10] = m_sendBuf->at(1) >> 8;  //读几位   (变量) >> 8
		Mod_Buff[11] = m_sendBuf->at(1) & 0x00ff;  //读几位  （变量）&  0x00ff
		len = 12;
		break;
	case Read_retain_word_register://0x03
		Mod_Buff[0] = Modbus_Flag >> 8;
		Mod_Buff[1] = Modbus_Flag & 0x00ff;
		Mod_Buff[2] = 0x00;  //协议标识符
		Mod_Buff[3] = 0x00;  //协议标识符
		Mod_Buff[4] = 0x00;  //字节长度
		Mod_Buff[5] = 0x06;  //字节长度
		Mod_Buff[6] = 0x01;  //站号
		Mod_Buff[7] = 0x03;  //功能码
		Mod_Buff[8] = m_sendBuf->at(0) >> 8;  //地址     （变量）>> 8
		Mod_Buff[9] = m_sendBuf->at(0) & 0x00ff;  //地址     （变量）&  0x00ff
		Mod_Buff[10] = m_sendBuf->at(1) >> 8;  //读几位   (变量) >> 8
		Mod_Buff[11] = m_sendBuf->at(1) & 0x00ff;  //读几位  （变量）&  0x00ff
		len = 12;
		break;
	case Read_Input_word_register://读取输入寄存器

		break;
	case Write_coil_bite_register://0x05写单个线圈寄存器
		Mod_Buff[0] = Modbus_Flag >> 8;     //事务标识符
		Mod_Buff[1] = Modbus_Flag & 0x00ff;  //事务标识符
		Mod_Buff[2] = 0x00;  //协议标识符tcp
		Mod_Buff[3] = 0x00;  //协议标识符tcp
		Mod_Buff[4] = 0x00;  //字节长度
		Mod_Buff[5] = 0x06;  //字节长度
		Mod_Buff[6] = 0x01;  //站号
		Mod_Buff[7] = 0x05;  //功能码
		Mod_Buff[8] = m_sendBuf->at(0) >> 8;       //地址     （变量）>> 8
		Mod_Buff[9] = m_sendBuf->at(0) & 0x00ff;   //地址     （变量）&  0x00ff
		if (m_sendBuf->at(1) == 1)
		{
			Mod_Buff[10] = 0xff;      //
			Mod_Buff[11] = 0x00;  //
		}
		else
		{
			Mod_Buff[10] = 0x00;      //
			Mod_Buff[11] = 0x00;  //
		}
		len = 12;
		break;
	case Write_retain_word_register://0x06 写单个保持寄存器,int型
		Mod_Buff[0] = Modbus_Flag >> 8;
		Mod_Buff[1] = Modbus_Flag & 0x00ff;
		Mod_Buff[2] = 0x00;  //协议标识符
		Mod_Buff[3] = 0x00;  //协议标识符
		Mod_Buff[4] = 0x00;  //字节长度
		Mod_Buff[5] = 0x2A;  //字节长度
		Mod_Buff[6] = 0x01;  //站号
		Mod_Buff[7] = 0x06;  //功能码
		Mod_Buff[8] = m_sendBuf->at(0) >> 8;  //地址     （变量）>> 8
		Mod_Buff[9] = m_sendBuf->at(0) & 0x00ff;  //地址     （变量）&  0x00ff
		Mod_Buff[10] = m_sendBuf->at(1) >> 8;  //data   (变量) >> 8
		Mod_Buff[11] = m_sendBuf->at(1) & 0x00ff;  //data  （变量）&  0x00ff
		len = 12;
		break;
	case Write_Multi_coil_register://写多个线圈寄存器

		break;
	case Write_Multi_retain_register://写多个保持寄存器float型，4字节
		Mod_Buff[0] = Modbus_Flag >> 8;
		Mod_Buff[1] = Modbus_Flag & 0x00ff;
		Mod_Buff[2] = 0x00;  //协议标识符
		Mod_Buff[3] = 0x00;  //协议标识符
		Mod_Buff[4] = (7 + m_sendBuf->at(1) * 4) >> 8;  //字节长度
		Mod_Buff[5] = 7 + m_sendBuf->at(1) * 4 & 0x00ff;  //字节长度
		Mod_Buff[6] = 0x01;  //站号
		Mod_Buff[7] = 0x10;  //功能码
		Mod_Buff[8] = m_sendBuf->at(0) >> 8;  //地址     （变量）>> 8
		Mod_Buff[9] = m_sendBuf->at(0) & 0x00ff;  //地址     （变量）&  0x00ff
		Mod_Buff[10] = m_sendBuf->at(1) * 2 >> 8;  //data   (变量) >> 8
		Mod_Buff[11] = m_sendBuf->at(1) * 2 & 0x00ff;  //data  （变量）&  0x00ff
		Mod_Buff[12] = m_sendBuf->at(1) * 4;
		//多少个GP*10 
		leng = m_sendBuf->at(1);
		for (int i = 0; i < leng; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				//mod_buff类型时 一个字节 数组     //m_sendBuff 类型时两个字节
				Mod_Buff[13 + i * 4 + j] = m_sendBuf->at(2 + i * 4 + j);
			}
		}
		len = 13 + m_sendBuf->at(1) * 4;
		break;
	default:
		break;
	}
	Obj_Modbus->Modbus_send(Obj_Modbus->ReturnSocket, Mod_Buff, len);//发送请求
	return Modbus_Flag;
}
int CROBOTPCtr::wait(int id, UINT timeout)
{
	long long start = XI_clock();

	map<int, vector<uint8_t>>::iterator it;

	while ((XI_clock() - start) < (timeout ? timeout : 200))
	{
		it = Modbus_rcvBuf->find(id);
		if (it != Modbus_rcvBuf->end() && it->second.at(0) == 1)return 0;

		Sleep(4);
	}
	return -1;
}

// x == y
template <typename T> bool CROBOTPCtr::almostEqual(T x, T y) {
	return (fabs(x - y) < std::numeric_limits<T>::epsilon()) ? true : false;
}
// x == y
template <typename T> bool CROBOTPCtr::almostEqual(T x, T y, uint32 precise) {
	return (fabs(x - y) < 1.0 / pow((double)10, (double)precise));
}
// x == 0
template <typename T> bool CROBOTPCtr::isZero(T x, uint32 precise) {
	return almostEqual(x, static_cast<T>(0), precise);
}
// v == 0
template <typename T> inline bool CROBOTPCtr::isZero(T const &v) { return v == 0; }
// float
// v == 0.0f
template <> inline bool CROBOTPCtr::isZero<float>(float const &v) {
	return fabs(v) < 0.000001;
}
// double
// v == 0.0
template <> inline bool CROBOTPCtr::isZero<double>(double const &v) {
	return fabs(v) < 0.000001;
}

