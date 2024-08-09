// CsvToXtensor.cpp
#include "CsvToXtensor.h"
#include <sstream>
#include <fstream>

/*
* 功能：去除字符串两端的空格、制表符和换行符
* 参数:
*   str: 要处理的字符串引用
*/
void Trim_crossing(std::string& str) 
{
    str.erase(0, str.find_first_not_of(" \t\r\n"));
    str.erase(str.find_last_not_of(" \t\r\n") + 1);
}