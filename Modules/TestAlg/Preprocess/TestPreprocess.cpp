#include "TestPreProcess.h"

/**
 * TestPreProcess构造函数
 * 功能：
 *   创建TestPreProcess对象时记录启动状态
 */
TestPreProcess::TestPreProcess()
{   
    LOG(INFO) << "TestPreProcess :: TestPreProcess  status:    Started." << std::endl;
}

/**
 * TestPreProcess析构函数
 * 功能：
 *   销毁TestPreProcess对象时执行的清理操作
 */
TestPreProcess::~TestPreProcess()
{

};

/**
 * 初始化图像后处理部分参数
 * 参数：
 *   p_pAlgParam - 触发算法参数
 * 功能：
 *   初始化TestPreProcess对象的算法参数
 */
void TestPreProcess::init(CSelfAlgParam* p_pAlgParam) 
{   
    LOG(INFO)<< "TestPreProcess :: init status : start."<<std::endl;
    if (!p_pAlgParam){
        LOG(ERROR)<< "The Trigger postprocess InitAlgorithn incoming parameter is empty"<<std::endl;
    }

    LOG(INFO)<< "TestPreProcess :: init status : finish!"<<std::endl;
}

/**
 * 执行图像后处理
 * 功能：
 *   获取输入数据，执行后处理逻辑，并设置卡口输出数据
 */
void TestPreProcess::execute(){

    LOG(INFO)<< "TestPreProcess::execute status: start."<<std::endl;

    // 输入数据获取
    // TestPreProcessResult = static_cast<CAlgResult *>(getCommonData()->trigger_result);

    // 输出数据设置
    // m_CommonData->trigger_result = TestPreProcessResult;


    LOG(INFO)<< "TestPreProcess::execute status : finish! " <<std::endl;
}