#include "stdafx.h"
#include "G_xml.h"


G_xml::G_xml()
{
	Decompose_Date = new vector<string>();
	Element_attribute = new vector<string>();
}


G_xml::~G_xml()
{
	delete Decompose_Date;
	delete Element_attribute;
}

int G_xml::Get_XML_date(const char * xmldate, int len)
{
	xml_Date = xmldate;
	string clear_Date;
	string Root;
	int time_Decompose=0;
	
	int first=xml_Date.find('<');
	int end_number = xml_Date.rfind('>');
	//删除< >之外的数据
	if (first < 0)return -1;
	clear_Date = xml_Date.substr(first, end_number- first + 1);
	while (true)
	{
		int first_a = clear_Date.find('<');
		int first_b = clear_Date.find('>');
		if (first_a < 0 || first_b < 0) break;
		if (first_a==0)
		{
			Decompose_Date->push_back(clear_Date.substr(first_a, first_b - first_a + 1));
			clear_Date = clear_Date.substr(first_b + 1);
		}
		else
		{
			Decompose_Date->push_back(clear_Date.substr(0, first_a ));
			clear_Date = clear_Date.substr(first_a);
		}
	}
	
	return xml_compose(Decompose_Date);
}

int G_xml::xml_compose(vector<string>* De_date)
{
	int len=De_date->size();
	//root 
	string root = De_date->at(0);
	 root.replace(0,1,"/");
	 root = "<"+ root;
	 if (root == De_date->at(len-1))
	 {
		 
	 }
	 else return -1;

	return 0;
}

int G_xml::Decompose_element(string Element_name, vector<string>* Out_Element_attribute)
{
	int len = Decompose_Date->size();
	Out_Element_attribute->clear();
	string sub_attribute;
	int line = compare_str(Decompose_Date, Element_name);
	int first = 0;
	int second = 0;
	int ifnull = -1;
	if (line < 0)return -1;
	int have_attribute = Decompose_Date->at(line).find("/");
	if (have_attribute>0)
	{
			Out_Element_attribute->push_back(0);//元素 值 0
	}
	else
	{
		int have_num = Decompose_Date->at(line + 1).find("/");
		if (have_num < 0)
		{
			Out_Element_attribute->push_back(Decompose_Date->at(line+1));//元素 值
		}
	}
	string infor = Decompose_Date->at(line);
	while (true)
	{
		first = infor.find("=");
		second = infor.find("\"");	
		if ((first < 0))break;
		if (second != first + 1)break;

		infor = infor.substr(second+1);
		second = infor.find("\"");
		
		Out_Element_attribute->push_back(infor.substr(0, second));//元素 值
		ifnull = 0;
		infor = infor.substr(second + 1);
	}
	return ifnull;
}

int G_xml::compare_str(vector<string>* Decompose, string if_haveString)
{
	int* result = NULL;
	bool ifCompate = false;
	int whileLine = 0;
	result = (int *)malloc(sizeof(int)*if_haveString.size());
	for (int i = 0; i < Decompose->size(); i++)
	{
		for (size_t j = 0; j < if_haveString.size(); j++)
		{
			result[j] = Decompose->at(i).find(if_haveString.at(j));
			if (result[j] < 0)
			{
				ifCompate = false;
				break;
			}
			ifCompate = true;
		}
		if (ifCompate)
		{
			whileLine = i;
			break;
		}
	}
	if (!ifCompate)
	{
		free(result);
		result = NULL;
		return -1;
	}
	for (size_t j = 0; j < if_haveString.size() - 1; j++)
	{
		if (result[j] + 1 != result[j + 1])
		{
			free(result);
			result = NULL;
			return -1;
		}
	}
	free(result);
	result = NULL;
	return whileLine;
}
