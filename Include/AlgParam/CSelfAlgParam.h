/*******************************************************
 文件：CSelfAlgParam.h
 作者：
 描述：算法内部使用的参数结构体
 版本：v2.0
 日期：2024-1-3
 *******************************************************/
#pragma once
#include <functional>
#include "CLidarParam.h"
#include "CCameraParam.h"
// #include "CStationParam.h"
#include "CAlgResult.h"
#include "json.hpp"

using AlgCallback = std::function<void(const CAlgResult&, void*)>;

struct CSelfAlgParam
{
	CSelfAlgParam() {}
	virtual ~CSelfAlgParam(){}
	
	CLidarParam     			m_tLidarParam;			//雷达参数结构体
	CCameraParam  				m_tCameraParam;			//相机参数结构体
	std::string					m_strRootPath;			//算法配置文件根目录路径

	// 点云算法参数
	std::string					m_strPcCfgPath;			//点云算法配置文件根目录路径
	int               			m_nLidarIndex;          //雷达序号，可根据该序号对指定雷达数据做处理

	// 视频算法参数
	int						    m_nVideoIndex;			//当前选取的相机序号，可根据该序号对指定相机数据做处理
	std::string 				m_strVideoCfgPath;		//视频检测权重路径配置文件
	int 						m_nCameraNum;			//相机个数
	std::vector<int> 			m_vecUsedCameraId;		//当前使用所有相机的Id
	std::vector<std::string>	m_vecVideoClass;		//视频检测类别

	//融合算法参数
	bool 						m_bFusion;				//是否融合 
	int 						m_nStationId;			//基站ID  
    uint16_t            		m_unFusionWaitTime;     //最长等待时间

	//融合跟踪算法参数
	std::string 				m_strFuTrCfgPath;					//融合跟踪权重路径配置文件
	std::string 				m_strCameraReflectLimitFilePath;	//融合跟踪权重路径配置文件
	std::string 	    		m_strFusionCfgPath;					//算法配置文件相对路径
	int							m_CameraId;	
	std::string 				m_strFusionConfPath;              
	std::vector<std::string> 	m_vecPcClass;
	std::vector<std::string> 	m_vecFusionClass;
	nlohmann::json 	  			m_fusion_parameter;
};
