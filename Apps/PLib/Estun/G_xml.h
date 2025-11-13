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
	int Get_XML_date(const char* xmldate , int len);//得到数据，分解数据
	int xml_compose(vector<string>* De_date);
	int Decompose_element(string element_name, vector<string>* Out_Element_attribute);//分解元素
public:
	vector<string> *Decompose_Date;   //分解原始数据为段落
	vector<string>* Element_attribute;//元素里的属性分解
	string  xml_Date;
	int len;
	
private:
	int compare_str(vector<string> *Decompose,string if_haveString);

};

