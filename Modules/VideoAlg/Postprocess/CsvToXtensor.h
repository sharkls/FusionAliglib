// CsvToXtensor.h
#ifndef CSV_TO_XTENSOR_H
#define CSV_TO_XTENSOR_H

#include <string>
#include <fstream>  // 添加这一行

#include "xtensor/xio.hpp"
#include "xtensor/xnpy.hpp"
#include "xtensor/xsort.hpp"
#include "xtensor/xmath.hpp"
#include "xtensor/xview.hpp"
#include "xtensor/xarray.hpp"
// #include "xtensor-blas/xlinalg.hpp"

void Trim_crossing(std::string& str);

/*
* 功能：将字符串转换为数值
* 参数:
*   str: 要转换的字符串
* 返回:
*   转换后的数值
*/
template <class Type>
Type stringToNum(const std::string& str)
{
    std::istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}

/*
* 功能：将一行CSV格式的字符串分割并转换为数值，存储到xtensor数组中的指定行
* 参数:
*   strSur: 输入的CSV格式字符串
*   cConChar: 分隔字符，例如逗号
*   video: xtensor数组，用于存储转换后的数值
*   idx: 指定要存储的行索引
*/
template <class Type>
void vStrSplit(std::string &strSur, char cConChar, xt::xarray<Type> &video, int idx)
{
    std::string::size_type pos1, pos2;
    pos1 = 0;
    pos2 = strSur.find(cConChar, 0);
    int i = 0;
    while(std::string::npos != pos2) // 当字符串 strSur 查询不到指定字符时
    {
        video(idx, i) = stringToNum<Type>(strSur.substr(pos1, pos2 - pos1));
        i += 1;
        pos1 = pos2 + 1;
        pos2 = strSur.find(cConChar, pos1);
    }
    video(idx, i) = stringToNum<Type>(strSur.substr(pos1, strSur.size()));      // 将最后一个字符转换为数字保存在video
}

/*
* 功能：从CSV文件中读取数据，并将其转换为xtensor数组
* 参数:
*   file_path: CSV文件的路径
*   max_row: 最大行数
*   max_col: 最大列数
* 返回:
*   包含CSV文件数据的xtensor数组
*/
template <class Type>
xt::xarray<Type> csv2Xtensor(std::string file_path, int max_row, int max_col)
{
    xt::xarray<Type> video = xt::zeros<Type>({max_row, max_col}); // max is 200
    int count = 0;
    std::ifstream csv_video;
    csv_video.open(file_path,std::ios::in);
    if (!csv_video.is_open())
    {
        std::cout<<"Open fusion Algconfig file error"<<std::endl;
    }
    
    std::string line;
    while (getline(csv_video, line))
    {
        Trim_crossing(line);                                // 去除字符串开头、结尾的 空格、制表符和换行符
        char s = ',';
        vStrSplit(line, s, video, count);                   // 将字符串以指定字符分割，并将分割后的字符串转换为数字存放之 video 中
        count += 1;
    }
    video = xt::view(video, xt::range(0, count));           // 获取 video 中前 count 个元素    
    return video;
}

#endif // CSV_TO_XTENSOR_H