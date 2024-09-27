#include "TestProcessing.h"

/**
 * TestProcessing构造函数
 * 功能：
 *   创建TestProcessing对象时记录启动状态
 */
TestProcessing::TestProcessing()
{   
    LOG(INFO) << "TestProcessing :: TestProcessing  status:    Started." << std::endl;
}

/**
 * TestProcessing析构函数
 * 功能：
 *   销毁TestProcessing对象时执行的清理操作
 */
TestProcessing::~TestProcessing()
{

};

/**
 * 初始化后处理部分参数
 * 参数：
 *   p_pAlgParam - 算法参数
 * 功能：
 *   初始化TestProcessing对象的算法参数
 */
void TestProcessing::init(CSelfAlgParam* p_pAlgParam) 
{   
    LOG(INFO)<< "TestProcessing :: init status : start."<<std::endl;
    if (!p_pAlgParam){
        LOG(ERROR)<< "The Trigger postprocess InitAlgorithn incoming parameter is empty"<<std::endl;
    }

    LOG(INFO)<< "TestProcessing :: init status : finish!"<<std::endl;
}

/**
 * 执行后处理
 * 功能：
 *   获取输入数据，执行后处理逻辑，并设置输出数据
 */
void TestProcessing::execute(){

    LOG(INFO)<< "TestProcessing::execute status: start."<<std::endl;

    // 输入数据获取
    int64_t TimeStamp = getCommonData()->TestStartTime;

    // 数据处理
    TimeStamp++;

    // 输出数据设置
    m_CommonData->TestStartTime = TimeStamp;

    LOG(INFO)<< "TestProcessing::execute status : finish! " <<std::endl;
}