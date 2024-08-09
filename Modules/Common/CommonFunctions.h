/*******************************************************
 文件：CommonFunctions.h
 作者：
 描述：公共函数定义，存放多个模块共用的一些函数，但若非必要，尽量不要往该文件中放
 版本：v1.0
 日期：2020-03-02
 *******************************************************/

#ifndef COMMONFUNCTIONS_H
#define COMMONFUNCTIONS_H

#include <iostream>
#include <sstream>
#include <limits>
#include <unistd.h>
#include <iomanip>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <math.h>
#include <time.h>
#include <typeinfo>

#include "glog/logging.h"

#ifndef F_OK
#define F_OK 0
#endif

/**
 * 将指定数据转化为字符串
 * 参数：p_tValue 待转为字符串的数据
 * 返回值：字符串
 */
template<class T> std::string ToStr(T p_tValue) 
{
    std::ostringstream l_outStream;
    // // l_outStream.flags(std::ios::fixed|std::ios::showpoint);//保证precision只控制小数位数，而不是整数位+小数位数；保留小数位后的0
    // l_outStream.flags(std::ios::fixed);//保证precision只控制小数位数，而不是整数位+小数位数；保留小数位后的0
    l_outStream.precision(std::numeric_limits<T>::digits10);//根据类型的精度设置该类型的的保留小数位数
    l_outStream << p_tValue;
    return l_outStream.str();
}

/**
 * 将数值转化为字符串
 * 参数：p_tValue 待转为字符串的数据
 * 参数：p_tRadix 数值进制
 * 返回值：字符串
 */
template<class T> std::string ToStr(T p_tValue, int p_tRadix) 
{
    std::ostringstream l_outStream;
    // l_outStream.flags(std::ios::fixed);//保证precision只控制小数位数，而不是整数位+小数位数；保留小数位后的0
    l_outStream.precision(std::numeric_limits<T>::digits10);//根据类型的精度设置该类型的的保留小数位数
    l_outStream << std::setbase(p_tRadix) << p_tValue;
    return l_outStream.str();
}

/**
 * 将字符串转为数字
*/
template<class T> static T& FromString(const std::string& s, T& p_tValue, int p_tRadix = 10)
{
	std::istringstream l_inStream(s);
    // l_inStream.flags(std::ios::fixed);//保证precision只控制小数位数，而不是整数位+小数位数；保留小数位后的0
    l_inStream.precision(std::numeric_limits<T>::digits10);//根据类型的精度设置该类型的的保留小数位数
    l_inStream >> std::setbase(p_tRadix) >> p_tValue;

    //若配置文件中bool值写的是0/1格式，则如上方方式直接转换即可，目前配置文件中bool值都是写的0/1格式，
    //若配置文件中bool值写的是true/false格式，则要按如下方式使用std::boolalpha做序列化bool类型
    // if(typeid(T) != typeid(bool))
    // {
    //     l_inStream.precision(std::numeric_limits<T>::digits10);//根据类型的精度设置该类型的的保留小数位数
	//     l_inStream >> std::setbase(p_tRadix) >> p_tValue;
    // }
    // else
    // {
    //     l_inStream >> std::boolalpha >> p_tValue;
    // }

	if (l_inStream.fail()) 
	{
		p_tValue = T();
	}
	return p_tValue;
}

template<class T> void VectorRelease(std::vector<T>&vec)
{
    std::vector<T>().swap(vec);
}


/**
 * 循环创建目录
 * 参数：p_strDirPath 待创建的目录路径（支持一次性创建多级目录）
 * 返回值：创建是否成功
 * 
    调用循环创建文件夹系统函数mkdir
    函数原型：int mkdir(const char *pathname, mode_t mode); 　　
    函数说明：mkdir()函数以mode方式创建一个以参数pathname命名的目录，mode定义新创建目录的权限。 　　
    返回值：若目录创建成功，则返回0；否则返回-1，并将错误记录到全局变量errno中。 　　
    mode方式：
    S_IRWXU 00700权限，代表该文件所有者拥有读，写和执行操作的权限 
    S_IRUSR(S_IREAD) 00400权限，代表该文件所有者拥有可读的权限 
    S_IWUSR(S_IWRITE) 00200权限，代表该文件所有者拥有可写的权限 
    S_IXUSR(S_IEXEC) 00100权限，代表该文件所有者拥有执行的权限 
    S_IRWXG 00070权限，代表该文件用户组拥有读，写和执行操作的权限 
    S_IRGRP 00040权限，代表该文件用户组拥有可读的权限 
    S_IWGRP 00020权限，代表该文件用户组拥有可写的权限 
    S_IXGRP 00010权限，代表该文件用户组拥有执行的权限 
    S_IRWXO 00007权限，代表其他用户拥有读，写和执行操作的权限 
    S_IROTH 00004权限，代表其他用户拥有可读的权限 
    S_IWOTH 00002权限，代表其他用户拥有可写的权限 
    S_IXOTH 00001权限，代表其他用户拥有执行的权限
 */
inline bool MyMakeDirs(const std::string& p_strDirPath)
{
	bool bIsExist = (0 == access(p_strDirPath.c_str(), F_OK));
	if(!bIsExist)
	{
		int l_nIsCreateOk = mkdir(p_strDirPath.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
		if(0 != l_nIsCreateOk)
		{
			LOG(ERROR) << "Failed to create directory:" << p_strDirPath << std::endl;
			return false;
		}
	}

    return true;
}

//ms级时间戳转成日期格式 yyyy-MM-dd HH:mm:ss:SSS
inline std::string TimestampMsToDateTime(unsigned long p_ulTimestampMs)
{
    int l_nMs = p_ulTimestampMs % 1000;  //求余的方式获取毫秒数
    time_t l_tS = p_ulTimestampMs / 1000;   //获取秒数
    // tm* l_tmDateTime = gmtime(&l_tS);       //由时间戳转换成tm结构体，内部含义查看该结构体的描述

	/* gmtime localtime 多线程不安全，采用localtime_r 函数*/

    tm l_tmGetTime;
    localtime_r(&l_tS, &l_tmGetTime);
    tm* l_tmDateTime = &l_tmGetTime;

    char l_cArrDateTime[64] = {0};
    sprintf(l_cArrDateTime, "%04d-%02d-%02d %02d:%02d:%02d:%03d",
         l_tmDateTime->tm_year + 1900, l_tmDateTime->tm_mon + 1, l_tmDateTime->tm_mday,
         l_tmDateTime->tm_hour, l_tmDateTime->tm_min, l_tmDateTime->tm_sec, l_nMs);

    return std::string(l_cArrDateTime);
}

//ms级时间戳 ，转化为年月日时分秒 毫秒 
inline bool TimestampToDate(uint64_t ulTimeStamp, uint16_t usOut[7])
{
    if (!usOut)
    {
        return false;
    }

    time_t l_tTimeS = ulTimeStamp / 1000;

    struct tm l_tmGetTime;
    localtime_r(&l_tTimeS, &l_tmGetTime);

    usOut[0] = (uint16_t)(l_tmGetTime.tm_year + 1900);
    usOut[1] = (uint16_t)(l_tmGetTime.tm_mon + 1);
    usOut[2] = (uint16_t)l_tmGetTime.tm_mday;
    usOut[3] = (uint16_t)l_tmGetTime.tm_hour;
    usOut[4] = (uint16_t)l_tmGetTime.tm_min;
    usOut[5] = (uint16_t)l_tmGetTime.tm_sec;
    usOut[6] = (uint16_t)(ulTimeStamp % 1000);

    return true;
}

//检查机器的大小端
inline bool IsSysLittleEndian() 
{
	union Check
	{
		int m_nVal;
		char m_cVal;
	}l_unionCheck;

	l_unionCheck.m_nVal = 1;
	return l_unionCheck.m_cVal == 1;
}

//ms级时间戳 ，转化为秒 分 时 日 月 年 LSW(即皮秒/2^32, 即1e12 / pow(2, 32))
inline bool TimestampToDate(uint64_t ulTimeStamp, uint8_t usOut[10])
{
    if (!usOut)
    {
        return false;
    }

    time_t l_tTimeS = ulTimeStamp / 1000;
    uint32_t l_unMs = ulTimeStamp % 1000;
	auto ll = pow(2, 32);
    uint32_t l_unLSW = l_unMs * 1e9 / (1e12 / pow(2, 32));

	if(IsSysLittleEndian())
	{
		std::cout << "little endian" << std::endl;
		uint8_t* l_pData = (uint8_t*)&l_unLSW;
		usOut[9] = (uint8_t)(*l_pData);
		usOut[8] = (uint8_t)(*(l_pData + 1));
		usOut[7] = (uint8_t)(*(l_pData + 2));
		usOut[6] = (uint8_t)(*(l_pData + 3));
	}
	else
	{
		std::cout << "big endian" << std::endl;
		memcpy(&usOut[6], &l_unLSW, 4);
	}

    struct tm l_tmGetTime;
    localtime_r(&l_tTimeS, &l_tmGetTime);

    // usOut[5] = (uint8_t)(l_tmGetTime.tm_year + 1900);
    usOut[5] = (uint8_t)(l_tmGetTime.tm_year);
    usOut[4] = (uint8_t)(l_tmGetTime.tm_mon + 1);
    usOut[3] = (uint8_t)l_tmGetTime.tm_mday;
    usOut[2] = (uint8_t)l_tmGetTime.tm_hour;
    usOut[1] = (uint8_t)l_tmGetTime.tm_min;
    usOut[0] = (uint8_t)l_tmGetTime.tm_sec;

    return true;
}


//二进制数组转16进制字符串
inline std::string BinToHex(const unsigned char *p_pData, size_t p_nSize)
{
	std::ostringstream l_strHex;
	l_strHex << std::hex << std::setfill('0');
	for (size_t i = 0; i < p_nSize; ++i)
    {
		l_strHex << std::setw(2)<<(static_cast<unsigned short>(p_pData[i]) & 0xff);
	}
	return l_strHex.str();
}

//16进制字符串转二进制数组
inline std::vector<unsigned char> HexToBin(const std::string &p_constRefStrHex)
{
	std::vector<unsigned char> l_vecDst;
	auto l_nLen = p_constRefStrHex.size();
	l_vecDst.reserve(l_nLen / 2);
	for (decltype(l_nLen) i = 0; i < l_nLen; i += 2)
	{
		unsigned int l_unDataTemp;
		std::istringstream l_strHex(p_constRefStrHex.substr(i, 2));
		l_strHex >> std::hex >> l_unDataTemp;
		l_vecDst.push_back(static_cast<unsigned char>(l_unDataTemp));
	}
	return l_vecDst;
}

/**
 * 弧度变角度
 * 参数：x 弧度值
 * 返回值：double  角度值
 */
inline double Rad2Degree(double x)
{
    return x * 180.0 / M_PI;
}

/**
 * 角度变弧度
 * 参数：x 角度值
 * 返回值：double  弧度值
 */
inline double Degree2Rad(double x)
{
    return x * M_PI / 180.0;
}

/**
 * 整数变成3字节数传出，大端模式，用于通信协议
 * 参数：x：入参 ，cOut ： 出参
 * 返回值：bool
 */
inline bool MakeUInt24Data(uint32_t x, uint8_t cOut[3])
{
    if (!cOut)
    {
        return false;
    }
    cOut[0] = (uint8_t)((x & 0x00ff0000) >> 16);
    cOut[1] = (uint8_t)((x & 0x0000ff00) >> 8);
    cOut[2] = (uint8_t)((x & 0x000000ff) >> 0);

    return true;
}

//分割字符串
inline void SplitString(const std::string& p_str, std::vector<std::string>& p_vecSplitStr, const std::string& p_strCharacter)
{
    std::string::size_type l_nPos1 = 0;
    std::string::size_type l_nPos2 = p_str.find(p_strCharacter);
    while(std::string::npos != l_nPos2)
    {
        p_vecSplitStr.push_back(p_str.substr(l_nPos1, l_nPos2-l_nPos1));

        l_nPos1 = l_nPos2 + p_strCharacter.size();
        l_nPos2 = p_str.find(p_strCharacter, l_nPos1);
    }
    if(l_nPos1 != p_str.length())
        p_vecSplitStr.push_back(p_str.substr(l_nPos1));
}


#endif