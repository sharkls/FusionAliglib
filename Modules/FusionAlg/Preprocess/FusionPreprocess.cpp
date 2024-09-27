#include "FusionPreprocess.h"


namespace fs = std::experimental::filesystem;

CFusionPreprocess::CFusionPreprocess()
{
    LOG(INFO) << "Fusion data preprocessing status :   start.";
}

CFusionPreprocess::~CFusionPreprocess()
{

}

void CFusionPreprocess::init(CSelfAlgParam* p_pAlgParam)
{
    LOG(INFO) << "CFusionPreprocess::init status :   start";

    // 1. 预处理参数获取
    p_PreFuTrAlgParam = *(static_cast<CSelfAlgParam *>(p_pAlgParam));

    // 2.加载配置文件路径
    cam_num_ = p_PreFuTrAlgParam.m_tCameraParam.unCameraCount();
    m_CameraParam = p_PreFuTrAlgParam.m_tCameraParam;
    img_Width = p_PreFuTrAlgParam.m_fusion_parameter["fusion_param"]["camera_raw_size"][0];
    img_High = p_PreFuTrAlgParam.m_fusion_parameter["fusion_param"]["camera_raw_size"][1];
    // m_vecCameraMatrix =  p_PreFuTrAlgParam.m_tCameraParam.vecCameraDev().vecInParameter();
    // m_vecDistCoeffs = p_PreFuTrAlgParam.m_tCameraParam.vecDistMatrix();
    // m_vecRotation = p_PreFuTrAlgParam.m_tCameraParam.vecRotateMatrix();
    // m_vecTranslation = p_PreFuTrAlgParam.m_tCameraParam.vecTranslationMatrix();
    LOG(INFO) << "CFusionPreprocess::init status :   finish!";
}

void CFusionPreprocess::execute()
{
    auto t_start = std::chrono::high_resolution_clock::now();
    LOG(INFO) << "CFusionPreprocess::execute  status :  started.";

    // 1. 融合预处理数据获取
    CAlgResult p_pSrcData = static_cast<CAlgResult>(getCommonData()->FuTrAlgdata);
    if (p_pSrcData.vecFrameResult().size() < 1)   // 核验融合数据的合法性
    {
        LOG(ERROR) << " CFusionPreprocess::execute Error : Fusion data is empty.";
    }

    // 2. 预处理输出数据初始化
    xt::xarray<float> pcResult;
    std::vector<xt::xarray<float>> vecPctoVideoResult;

    // 3. 遍历所有帧数据，构建点云结果数据和图像结果数据
    int _camera_num = 0;
    for (int i=0; i< p_pSrcData.vecFrameResult().size(); i++)
    {
        LOG(INFO) << " p_pSrcData.vecFrameResult()[i].eDataType() : " << (int)p_pSrcData.vecFrameResult()[i].eDataType();

        // 3.1 融合原始数据保存
        if(p_PreFuTrAlgParam.m_fusion_parameter["fusion_param"]["save_raw_data"])
        {
            std::string l_sOutPutPath = p_PreFuTrAlgParam.m_strRootPath + "/SaveData/";
            save_input_to_npy(l_sOutPutPath,  p_pSrcData);
        }

        // 3.2 点云获取数据转换
        if (p_pSrcData.vecFrameResult()[i].eDataType() == DATA_TYPE_PC_RESULT)       //3=点云类型
        {
            PcAlgResTransfer(&p_pSrcData.vecFrameResult()[i], pcResult);
        }
        // 3.3 图像获取数据转换
        else if (p_pSrcData.vecFrameResult()[i].eDataType() == DATA_TYPE_VIDEO_RESULT)  // 6=视频类型
        {
            _camera_num ++;
            xt::xarray<float> video;

            // 3.3.1 图像数据转换
            video = VideoAlgResTransfer(& p_pSrcData.vecFrameResult()[i], _camera_num);

            // 3.3.2 根据相机反映射配置，对相机检测目标进行过滤
            auto col = xt::view(video, xt::all(), 10);
            col = xt::where(col >= 1.0, 0.98, col);
            auto filter_box_x_index = xt::where(xt::col(video, 0) < 0.001f && xt::col(video, 0) >= 0.0f)[0];
            video = xt::view(video, xt::drop(filter_box_x_index));
            m_vecVideoResult.push_back(video);
        }
    }

    // 4. （多相机目标与对应点云目标进行Iou匹配）点云投影至相机坐标系下的目标和对应相机目标

    // 获取点云目标框投影至各相机上
    for(size_t i  = 0; i < cam_num_; i++)
    {
        vecPctoVideoResult.push_back(PcToVideoTransform(pcResult, m_vecVideoResult, i));
    }
    // 4. 融合跟踪预处理数据输出
    m_CommonData->m_vecVideoXarrayResult = m_vecVideoResult;
    m_CommonData->m_fPcXarrayResult = pcResult;
    setCommonData(m_CommonData);
    LOG(INFO) << "CFusionPreprocess::execute  status :  finish!";

    auto t_end = std::chrono::high_resolution_clock::now();
    auto latency_all = (std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_end - t_start).count())/1000;
}

void CFusionPreprocess::PcAlgResTransfer(CFrameResult *p_pPcResult, xt::xarray<float>& PcResult)
{
    // x,y,w,l,theta(角度),z,h,label,speed, id, score, [x_min,y_min,x_max,y_max,] data_source, [data_channel]
    PcResult = xt::zeros<float>({int(p_pPcResult->vecObjectResult().size()), 17});
    LOG(INFO) << "PcAlgResTransfer Lidar Nums: " << p_pPcResult->vecObjectResult().size();
    for (int i = 0; i < int(p_pPcResult->vecObjectResult().size()); i++)
    {   
        PcResult(i, 0) = float(p_pPcResult->vecObjectResult()[i].sXCoord()) / 100;         // x
        PcResult(i, 1) = float(p_pPcResult->vecObjectResult()[i].sYCoord()) / 100;         // y
        PcResult(i, 2) = float(p_pPcResult->vecObjectResult()[i].usWidth()) / 100;         // w
        PcResult(i, 3) = float(p_pPcResult->vecObjectResult()[i].usLength()) / 100;        // l
        PcResult(i, 4) = float(p_pPcResult->vecObjectResult()[i].sCourseAngle()); //* 180 / PI; // theta
        PcResult(i, 5) = float(p_pPcResult->vecObjectResult()[i].sZCoord()) / 100;         // z
        PcResult(i, 6) = float(p_pPcResult->vecObjectResult()[i].usHeight()) / 100;        // h
        PcResult(i, 7) = PcClass2label(p_pPcResult->vecObjectResult()[i].strClass(), p_PreFuTrAlgParam.m_fusion_parameter);  // label
        PcResult(i, 8) = float(p_pPcResult->vecObjectResult()[i].sSpeed());             // speed
        PcResult(i, 9) = p_pPcResult->vecObjectResult()[i].usTargetId();                 // id
        PcResult(i, 10) = float(p_pPcResult->vecObjectResult()[i].fPcConfidence()) / 100; // score  
        PcResult(i, 11) = float(-1000);     // x_min
        PcResult(i, 12) = float(-1000);     // y_min
        PcResult(i, 13) = float(-1000);     // x_max
        PcResult(i, 14) = float(-1000);     // y_max
        PcResult(i, 15) = float(2);// 2;     // data_source ..ucSource;       // CAlgResult.idl在融合结果中使用，每个目标的来源，0—激光，1—视频，2-毫米波，3—视频和激光融合
        PcResult(i, 16) = 0;     // data_channel
    }
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

// 新增函数：将雷达目标转换为图像上的二维目标框
xt::xarray<float> CFusionPreprocess::PcToVideoTransform(xt::xarray<float>& pc, std::vector<xt::xarray<float>> vecVideoResult, size_t index_cam)
{   
    size_t num_boxes = pc.shape()[0];
    // image_boxes = xt::zeros<float>({num_boxes, 4}); // 每个目标框包含4个值：x_min, y_min, x_max, y_max

    // 获取相机内参矩阵K
    xt::xarray<float> K = xt::adapt(m_CameraParam.vecCameraDev()[index_cam].vecInParameter(), {3, 3});

    // 获取相机旋转矩阵R
    xt::xarray<float> R = xt::adapt(m_CameraParam.vecCameraDev()[index_cam].vecRotateMatrix(), {3, 3});

    // 获取相机平移向量T
    xt::xarray<float> T = xt::adapt(m_CameraParam.vecCameraDev()[index_cam].vecTranslationMatrix(), {3, 1});

    // 获取畸变系数
    std::vector<float> dist_coeffs = m_CameraParam.vecCameraDev()[index_cam].vecDistMatrix();

    // 构建外参矩阵RT
    xt::xarray<float> RT = xt::hstack(xt::xtuple(R, T));

    for (size_t i = 0; i < num_boxes; ++i)
    {
        // 获取目标框的中心坐标和尺寸
        float x = pc(i, 0);
        float y = pc(i, 1);
        float z = pc(i, 5);
        float w = pc(i, 2);
        float h = pc(i, 6);
        float l = pc(i, 3);

        // 计算目标框的八个顶点
        std::vector<xt::xarray<float>> corners = {
            {x - w / 2, y - l / 2, z - h / 2, 1.0},
            {x + w / 2, y - l / 2, z - h / 2, 1.0},
            {x - w / 2, y + l / 2, z - h / 2, 1.0},
            {x + w / 2, y + l / 2, z - h / 2, 1.0},
            {x - w / 2, y - l / 2, z + h / 2, 1.0},
            {x + w / 2, y - l / 2, z + h / 2, 1.0},
            {x - w / 2, y + l / 2, z + h / 2, 1.0},
            {x + w / 2, y + l / 2, z + h / 2, 1.0}
        };

        float x_min = std::numeric_limits<float>::max();
        float y_min = std::numeric_limits<float>::max();
        float x_max = std::numeric_limits<float>::lowest();
        float y_max = std::numeric_limits<float>::lowest();

        for (const auto& corner : corners)
        {
            // 使用相机内外参将三维坐标转换为二维图像坐标
            xt::xarray<float> point_2d = xt::linalg::dot(K, xt::linalg::dot(RT, corner));

            // 归一化
            float u = point_2d(0) / point_2d(2);
            float v = point_2d(1) / point_2d(2);

            // 畸变校正
            float r2 = u * u + v * v;
            float r4 = r2 * r2;
            float r6 = r4 * r2;

            float k1 = dist_coeffs[0];
            float k2 = dist_coeffs[1];
            float p1 = dist_coeffs[2];
            float p2 = dist_coeffs[3];
            float k3 = dist_coeffs[4];

            float x_distorted = u * (1 + k1 * r2 + k2 * r4 + k3 * r6) + 2 * p1 * u * v + p2 * (r2 + 2 * u * u);
            float y_distorted = v * (1 + k1 * r2 + k2 * r4 + k3 * r6) + p1 * (r2 + 2 * v * v) + 2 * p2 * u * v;

            // 更新目标框的最小和最大值
            x_min = std::min(x_min, x_distorted);
            y_min = std::min(y_min, y_distorted);
            x_max = std::max(x_max, x_distorted);
            y_max = std::max(y_max, y_distorted);
        }

        // 判断目标框是否在图像范围内
        if (x_max < 0 || x_min > img_Width || y_max < 0 || y_min > img_High)
        {
            // 目标框完全不在图像范围内，跳过该目标
            continue;
        }

        // 调整目标框的顶点值，使其在图像范围内
        x_min = std::max(0.0f, x_min);
        y_min = std::max(0.0f, y_min);
        x_max = std::min(static_cast<float>(img_Width), x_max);
        y_max = std::min(static_cast<float>(img_High), y_max);

        // 将调整后的目标框顶点值赋值给video_boxes
        pc(i, 11) = x_min;
        pc(i, 12) = y_min;
        pc(i, 13) = x_max;
        pc(i, 14) = y_max;
    }

    // 匹配点云和相机目标，并进行目标信息融合
    MatchAndFuseTargets(pc, vecVideoResult, index_cam);
    // LOG(INFO) << "CFusionPreprocess::PcToVideoTransform ---- status : end!";
    return pc;
}

void CFusionPreprocess::MatchAndFuseTargets(xt::xarray<float>& pc, std::vector<xt::xarray<float>> vecVideoResult, size_t index_cam)
{   
    // LOG(INFO) << "CFusionPreprocess::MatchAndFuseTargets ---- status : start!";

    std::vector<xt::xarray<float>> m_vecNoMatchVideoResult;
    // 构建匈牙利算法的代价矩阵
    size_t num_lidar = pc.shape()[0];
    size_t num_camera = vecVideoResult[index_cam].shape()[0];
    // xt::xarray<float> cost_matrix({num_lidar, num_camera});

    // 提取点云目标和图像目标的边界框
    xt::xarray<float> lidar_boxes = xt::view(pc, xt::all(), xt::range(11, 15));
    xt::xarray<float> camera_boxes = xt::view(vecVideoResult[index_cam], xt::all(), xt::range(11, 15));

    // 调用 iou_jit_new 计算 IoU
    auto [overlaps, overlaps_new] = FunctionHub::iou_jit_new(lidar_boxes, camera_boxes, 1e-5);

    // 使用 IoU 计算成本矩阵
    xt::xarray<float> cost_matrix = 1.0f - overlaps;

    // 执行匈牙利算法
    xt::xarray<int> assignment = run_hungarian_match(cost_matrix);

    // 将匹配上的图像目标检测框信息赋值给点云目标，并将点云目标的data_source改为3
    std::vector<bool> matched_camera_indices(num_camera, false);
    for (size_t j = 0; j < assignment.shape(0); ++j)
    {
        int lidar_idx = assignment(j, 0);
        int camera_idx = assignment(j, 1);

        if (camera_idx != -1)
        {
            // 复制图像目标检测框信息
            pc(lidar_idx, 11) = vecVideoResult[index_cam](j, 11);
            pc(lidar_idx, 12) = vecVideoResult[index_cam](j, 12);
            pc(lidar_idx, 13) = vecVideoResult[index_cam](j, 13);
            pc(lidar_idx, 14) = vecVideoResult[index_cam](j, 14);

            if(pc(lidar_idx, 7) != vecVideoResult[index_cam](j, 7))
            {
                pc(lidar_idx, 7) = vecVideoResult[index_cam](j, 7);   // 如果不等，用相机类别替换点云目标类别
            }
            
            pc(lidar_idx, 15) = 4;                                      // 目标为融合后的结果
            matched_camera_indices[camera_idx] = true;
        }
    }

    // 保存未匹配上的图像目标
    std::vector<size_t> unmatched_indices;
    for(size_t k = 0; k < num_camera; ++k)
    {
        if (!matched_camera_indices[k])
        {
            unmatched_indices.push_back(k);
        }
    }

    if (!unmatched_indices.empty())
    {
        xt::xarray<float> no_match_result = xt::view(vecVideoResult[index_cam], xt::keep(unmatched_indices));
        m_vecNoMatchVideoResult.push_back(no_match_result);
    }
    // LOG(INFO) << "CFusionPreprocess::MatchAndFuseTargets ---- status : end!";
}