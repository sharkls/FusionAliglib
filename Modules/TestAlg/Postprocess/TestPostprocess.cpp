#include "TestPostProcess.h"

/**
 * TestPostProcess构造函数
 * 功能：
 *   创建TestPostProcess对象时记录启动状态
 */
TestPostProcess::TestPostProcess()
{   
    LOG(INFO) << "TestPostProcess :: TestPostProcess  status:    Started." << std::endl;
}

/**
 * TestPostProcess析构函数
 * 功能：
 *   销毁TestPostProcess对象时执行的清理操作
 */
TestPostProcess::~TestPostProcess()
{

};

/**
 * 初始化图像后处理部分参数
 * 参数：
 *   p_pAlgParam - 触发算法参数
 * 功能：
 *   初始化TestPostProcess对象的算法参数
 */
void TestPostProcess::init(CSelfAlgParam* p_pAlgParam) 
{   
    LOG(INFO)<< "TestPostProcess :: init status : start."<<std::endl;
    if (!p_pAlgParam){
        LOG(ERROR)<< "The Trigger postprocess InitAlgorithn incoming parameter is empty"<<std::endl;
    }

    LOG(INFO)<< "TestPostProcess :: init status : finish!"<<std::endl;
}

/**
 * 执行图像后处理
 * 功能：
 *   获取输入数据，执行后处理逻辑，并设置卡口输出数据
 */
void TestPostProcess::execute(){

    LOG(INFO)<< "TestPostProcess::execute status: start."<<std::endl;

    // 输入数据获取
    // TestPostProcessResult = static_cast<CAlgResult *>(getCommonData()->trigger_result);

    // 出数据设置
    // m_CommonData->trigger_result = TestPostProcessResult;

    LOG(INFO)<< "TestPostProcess::execute status : finish! " <<std::endl;
}