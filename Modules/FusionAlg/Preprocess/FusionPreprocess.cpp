#include "FusionPreprocess.h"
#include "Log/glog/logging.h"

namespace fs = std::experimental::filesystem;

CFusionPreprocess::CFusionPreprocess()
{
    LOG(INFO) << "Fusion_Tracking data preprocessing status :   start.";
}

CFusionPreprocess::~CFusionPreprocess()
{

}

void CFusionPreprocess::init(CSelfAlgParam* p_pAlgParam)
{
    LOG(INFO) << "CFusionPreprocess::init status :   start";

    // 1. 预处理参数获取
    p_PreFuTrAlgParam = *(static_cast<CSelfAlgParam *>(p_pAlgParam));
    LOG(INFO) << "CFusionPreprocess::init status :   finish!";
}

void CFusionPreprocess::execute()
{
    auto t_start = std::chrono::high_resolution_clock::now();
    LOG(INFO) << "CFusionPreprocess::execute  status :  started.";

    // 1. 融合预处理数据获取
    CAlgResult p_pSrcData = static_cast<CAlgResult>(getCommonData()->FuTrAlgdata);
    if (p_pSrcData.vecFrameResult().size() < 1)             // 核验融合跟踪数据的合法性
    {
        LOG(ERROR) << " CFusionPreprocess::execute Error : FusionTrack data is empty.";
    }

    // 2. 预处理输出数据初始化
    xt::xarray<float> pc;
    std::vector<xt::xarray<float>> vecVideoResult;

    // 3. 遍历所有帧数据，构建点云结果数据和图像结果数据
    int _camera_num = 0;
    for (int i=0; i< p_pSrcData.vecFrameResult().size(); i++)
    {
        LOG(INFO) << " p_pSrcData.vecFrameResult()[i].eDataType() : " << (int)p_pSrcData.vecFrameResult()[i].eDataType();

        if(p_PreFuTrAlgParam.m_fusion_parameter["fusion_param"]["save_raw_data"])
        {
            std::string l_sOutPutPath = p_PreFuTrAlgParam.m_strRootPath + "/SaveData/";
            save_input_to_npy(l_sOutPutPath,  p_pSrcData);
        }

        // 3.1 点云获取数据转换
        if (p_pSrcData.vecFrameResult()[i].eDataType() == DATA_TYPE_PC_RESULT)       //3=点云类型
        {
            pc = PcAlgResTransfer(& p_pSrcData.vecFrameResult()[i]);
        }
            // 3.2 图像获取数据转换
        else if (p_pSrcData.vecFrameResult()[i].eDataType() == DATA_TYPE_VIDEO_RESULT)  // 6=视频类型
        {
            _camera_num ++;
            xt::xarray<float> video;

            // 3.2.1 图像数据转换
            video = VideoAlgResTransfer(& p_pSrcData.vecFrameResult()[i], _camera_num);

            // 3.2.2 根据相机反映射配置，对相机检测目标进行过滤
            auto col = xt::view(video, xt::all(), 10);
            col = xt::where(col >= 1.0, 0.98, col);
            auto filter_box_x_index = xt::where(xt::col(video, 0) < 0.001f && xt::col(video, 0) >= 0.0f)[0];
            video = xt::view(video, xt::drop(filter_box_x_index));
            vecVideoResult.push_back(video);
        }
    }

    // 4. 融合跟踪预处理数据输出
    m_CommonData->m_vecVideoXarrayResult = vecVideoResult;
    m_CommonData->m_fPcXarrayResult = pc;
    setCommonData(m_CommonData);
    LOG(INFO) << "CFusionPreprocess::execute  status :  finish!";

    auto t_end = std::chrono::high_resolution_clock::now();
    auto latency_all = (std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_end - t_start).count())/1000;
}

xt::xarray<float> CFusionPreprocess::PcAlgResTransfer(CFrameResult *p_pPcResult)
{
    // x,y,w,l,theta(角度),z,h,label,speed, id, score, [x_min,y_min,x_max,y_max,] data_source, [data_channel]
    xt::xarray<float> lidar_box = xt::zeros<float>({int(p_pPcResult->vecObjectResult().size()), 17});
    LOG(INFO) << "PcAlgResTransfer Lidar Nums: " << p_pPcResult->vecObjectResult().size();
    for (int i = 0; i < int(p_pPcResult->vecObjectResult().size()); i++)
    {
        lidar_box(i, 0) = float(p_pPcResult->vecObjectResult()[i].sXCoord()) / 100;         // x
        lidar_box(i, 1) = float(p_pPcResult->vecObjectResult()[i].sYCoord()) / 100;         // y
        lidar_box(i, 2) = float(p_pPcResult->vecObjectResult()[i].usWidth()) / 100;         // w
        lidar_box(i, 3) = float(p_pPcResult->vecObjectResult()[i].usLength()) / 100;        // l
        lidar_box(i, 4) = float(p_pPcResult->vecObjectResult()[i].sCourseAngle()); //* 180 / PI; // theta
        lidar_box(i, 5) = float(p_pPcResult->vecObjectResult()[i].sZCoord()) / 100;         // z
        lidar_box(i, 6) = float(p_pPcResult->vecObjectResult()[i].usHeight()) / 100;        // h
        lidar_box(i, 7) = PcClass2label(p_pPcResult->vecObjectResult()[i].strClass(), p_PreFuTrAlgParam.m_fusion_parameter);  // label
        lidar_box(i, 8) = float(p_pPcResult->vecObjectResult()[i].sSpeed());             // speed
        lidar_box(i, 9) = p_pPcResult->vecObjectResult()[i].usTargetId();                 // id
        lidar_box(i, 10) = float(p_pPcResult->vecObjectResult()[i].fPcConfidence()) / 100; // score
        lidar_box(i, 11) = float(-1000);     // x_min
        lidar_box(i, 12) = float(-1000);     // y_min
        lidar_box(i, 13) = float(-1000);     // x_max
        lidar_box(i, 14) = float(-1000);     // y_max
        lidar_box(i, 15) = float(2);// 2;     // data_source ..ucSource;       // CAlgResult.idl在融合结果中使用，每个目标的来源，0—激光，1—视频，2-毫米波，3—视频和激光融合
        lidar_box(i, 16) = 0;     // data_channel
    }
    return lidar_box;
}

xt::xarray<float> CFusionPreprocess::VideoAlgResTransfer(CFrameResult *p_pVideoResult, int channel)
{
    xt::xarray<float> each_video_for_fusion_box;
    int each_video_box_num = p_pVideoResult->vecObjectResult().size();
    LOG(INFO) << "each_video_box_num :" << each_video_box_num ;
    if (each_video_box_num == 0)        // 该路相机的目标框集个数为0
    {
        each_video_for_fusion_box = xt::empty<float>({0, 17});
    }
    else
    {
        each_video_for_fusion_box = xt::zeros<float>({each_video_box_num, 17});
        // 遍历一路相机的所有目标框
        for (int j = 0; j < each_video_box_num; j++)
        {                                                                                                                                                // 遍历某一路相机的所有目标框
            each_video_for_fusion_box(j, 0) = float(p_pVideoResult->vecObjectResult()[j].sXCoord()) / 100;                                                        // x
            each_video_for_fusion_box(j, 1) = float(p_pVideoResult->vecObjectResult()[j].sYCoord()) / 100;                                                        // y
            each_video_for_fusion_box(j, 2) = float(p_pVideoResult->vecObjectResult()[j].usWidth()) / 100;                                                        // w
            each_video_for_fusion_box(j, 3) = float(p_pVideoResult->vecObjectResult()[j].usLength()) / 100;                                                       // l
            each_video_for_fusion_box(j, 4) = float(p_pVideoResult->vecObjectResult()[j].sCourseAngle()) / 100;                                                         // theta
            each_video_for_fusion_box(j, 5) = float(p_pVideoResult->vecObjectResult()[j].sZCoord()) / 100;                                                        // z
            each_video_for_fusion_box(j, 6) = float(p_pVideoResult->vecObjectResult()[j].usHeight()) / 100;                                                       // h
            each_video_for_fusion_box(j, 7) = VideoClass2label(p_pVideoResult->vecObjectResult()[j].strClass(), p_PreFuTrAlgParam.m_fusion_parameter); // label
            each_video_for_fusion_box(j, 8) = float(p_pVideoResult->vecObjectResult()[j].sSpeed()) / 100;
            each_video_for_fusion_box(j, 9) = p_pVideoResult->vecObjectResult()[j].usTargetId();
            each_video_for_fusion_box(j, 10) = float(p_pVideoResult->vecObjectResult()[j].fVideoConfidence()) / 100;
            each_video_for_fusion_box(j, 11) = p_pVideoResult->vecObjectResult()[j].fTopLeftX();
            each_video_for_fusion_box(j, 12) = p_pVideoResult->vecObjectResult()[j].fTopLeftY();
            each_video_for_fusion_box(j, 13) = p_pVideoResult->vecObjectResult()[j].fBottomRightX();
            each_video_for_fusion_box(j, 14) = p_pVideoResult->vecObjectResult()[j].fBottomRightY();
            each_video_for_fusion_box(j, 15) = p_pVideoResult->vecObjectResult()[j].ucSource();
            each_video_for_fusion_box(j, 16) = float(channel);
        }
    }
    return each_video_for_fusion_box;
}

int CFusionPreprocess::PcClass2label(std::string &p_strClass, nlohmann::json fusion_parameter)
{
    for (int i = 0; i < fusion_parameter["fusion_param"]["m_strPcClass"].size(); i++)
    {
        if (p_strClass == fusion_parameter["fusion_param"]["m_strPcClass"][i])
        {
            return fusion_parameter["fusion_param"]["lidar_class_to_fusion_class"][std::to_string(i)];
        }
    }
    return 0;
}

int CFusionPreprocess::VideoClass2label( std::string &p_strClass, nlohmann::json fusion_parameter)
{
    for (int i = 0; i < fusion_parameter["fusion_param"]["m_strVideoClass"].size(); i++)
    {
        if (p_strClass == fusion_parameter["fusion_param"]["m_strVideoClass"][i])
        {
            return fusion_parameter["fusion_param"]["video_class_to_fusion_class"][std::to_string(i)];
        }
    }
    return 0;
}

void CFusionPreprocess::save_input_to_npy(std::string path, CAlgResult outputAlgResult){

    std::string frameName;
    int pc_flag = 0;
    string frameID;
    string PcFrameID;
    string VideoFrameID;
    for(int i = 0 ; i < outputAlgResult.vecFrameResult().size(); i++)
    {
        if (outputAlgResult.vecFrameResult()[i].eDataType() == DATA_TYPE_PC_RESULT)
        {
            pc_flag = 1;
            PcFrameID = std::to_string(outputAlgResult.vecFrameResult()[i].unFrameId());
        }else if (outputAlgResult.vecFrameResult()[i].eDataType() == DATA_TYPE_VIDEO_RESULT)
        {
            VideoFrameID = std::to_string(outputAlgResult.vecFrameResult()[i].unFrameId());
        }
    }
    frameID = VideoFrameID;
    if(pc_flag){
        frameID = PcFrameID;
    }
    for(int i = 0 ; i < outputAlgResult.vecFrameResult().size(); i++)
    {
        char frameNum[7];
        if (outputAlgResult.vecFrameResult()[i].eDataType() == DATA_TYPE_PC_RESULT)
        {
            snprintf(frameNum, sizeof(frameNum), "%06d", outputAlgResult.vecFrameResult()[0].unFrameId());
            frameName = std::string(path + "PcResults/Frame_" + frameID + ".npy");

        }
        else if (outputAlgResult.vecFrameResult()[i].eDataType() == DATA_TYPE_VIDEO_RESULT)
        {
            snprintf(frameNum, sizeof(frameNum), "%06d", outputAlgResult.vecFrameResult()[0].unFrameId());
            int cnum = outputAlgResult.vecFrameResult()[i].tCameraSupplement().ucCameraId();
            frameName = std::string(path + "VL_"+ std::to_string(cnum) + "/Frame_" + frameID + ".npy");
        }

        int size_out = outputAlgResult.vecFrameResult()[i].vecObjectResult().size();
        xt::xarray<float> all_boxes = xt::zeros<float>({size_out, 12});
        for (int j = 0; j < outputAlgResult.vecFrameResult()[i].vecObjectResult().size(); j++)
        {
            all_boxes(j, 0) = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].sXCoord()/100.0;
            all_boxes(j, 1) = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].sYCoord()/100.0;
            all_boxes(j, 2) = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].sZCoord()/100.0;
            all_boxes(j, 3) = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].usWidth()/100.0;
            all_boxes(j, 4) = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].usLength()/100.0;
            all_boxes(j, 5) = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].usHeight()/100.0;
            all_boxes(j, 6) = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].sSpeed()/100.0;
            all_boxes(j, 7) = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].sCourseAngle();
            std::string cls1 = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].strClass() ;
            all_boxes(j, 8) = VideoClass2label(cls1, p_PreFuTrAlgParam.m_fusion_parameter);
            all_boxes(j, 9) = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].ucSource();
            all_boxes(j, 10) = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].fPcConfidence();
            all_boxes(j, 11) = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].usTargetId();
        }

        fs::path pathObj = (frameName);
        if(!fs::exists(pathObj.parent_path()))
        {
            fs::create_directories(pathObj.parent_path());
        }
        xt::dump_npy(frameName, all_boxes);
    }
}
