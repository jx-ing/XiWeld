#pragma once
#include <vector>
#include <string>
#include <map>
using namespace std;
class G_xml
{
public:
	G_xml();
	~G_xml();
	int Get_XML_date(const char* xmldate , int len);//�õ����ݣ��ֽ�����
	int xml_compose(vector<string>* De_date);
	int Decompose_element(string element_name, vector<string>* Out_Element_attribute);//�ֽ�Ԫ��
public:
	vector<string> *Decompose_Date;   //�ֽ�ԭʼ����Ϊ����
	vector<string>* Element_attribute;//Ԫ��������Էֽ�
	string  xml_Date;
	int len;
	
private:
	int compare_str(vector<string> *Decompose,string if_haveString);

};

