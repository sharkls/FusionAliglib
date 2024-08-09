#include "Video_detection_postprocessor.h"
#include "Log/glog/logging.h"
#include "CSelfAlgParam.h"
#include <cstdio>
#include <cstring>
#include "xtensor/xview.hpp"
#include "tinyxml2.h"

using namespace tinyxml2;

/**
 * BLH_To_XYZ 函数
 * 功能：将BLH坐标（经纬度）转换为XYZ坐标（笛卡尔坐标系）
 * 参数：
 *   lon - 经度
 *   lat - 纬度
 *   oriLon - 原点经度
 *   oriLat - 原点纬度
 *   oriAngle - 原点角度
 *   targetX - 转换后的X坐标
 *   targetY - 转换后的Y坐标
 */
inline void BLH_To_XYZ(
        const double lon, const double lat, const double oriLon, double oriLat, const float oriAngle, float &targetX, float &targetY)
    {
        float l_radian = oriAngle * M_PI / 180;
        double R_a = 6378137.00;   // 地球长半轴
        double R_b = 6356752.3142; // 地球短半轴
        double l_tmpx = (lon - oriLon) * M_PI / 180 * R_a * cos(oriLat * M_PI / 180);
        double l_tmpy = (lat - oriLat) * M_PI / 180 * R_b;
        targetX = l_tmpx * cos(l_radian) - (l_tmpy)*sin(l_radian);
        targetY = l_tmpx * sin(l_radian) + (l_tmpy)*cos(l_radian);

        return;
    }


/*
* 功能：构造函数，初始化视频检测后处理器并记录日志
*/
CVPost :: CVPost()
{   
    LOG(INFO) << "CVPost::CVPost  status:    Started." << std::endl;
}

/*
* 功能：析构函数，用于清理视频检测后处理器
*/
CVPost :: ~CVPost()
{

};

/*
* 功能：初始化视频检测后处理器，设置算法参数，读取配置文件
* 参数:
*   p_pAlgParam: 指向算法参数的指针
*/
void CVPost :: init(CSelfAlgParam* p_pAlgParam) 
{   
    LOG(INFO)<< "CVPost::init status : start!"<<std::endl;
    if (!p_pAlgParam){
        LOG(ERROR)<< "The Video postprocess InitAlgorithn incoming parameter is empty"<<std::endl;
    }

    // 1.配置文件读取
    LOG(INFO)<< p_pAlgParam->m_strRootPath << p_pAlgParam->m_strVideoCfgPath << std::endl;
    YAML::Node video_cfg = YAML::LoadFile(p_pAlgParam->m_strRootPath + p_pAlgParam->m_strVideoCfgPath);
    p_pAlgParam->m_vecVideoClass = {video_cfg["YOLOV5_CLASS_NAMES"][0].as<std::string>(),
              video_cfg["YOLOV5_CLASS_NAMES"][1].as<std::string>(), video_cfg["YOLOV5_CLASS_NAMES"][2].as<std::string>(),
              video_cfg["YOLOV5_CLASS_NAMES"][3].as<std::string>(), video_cfg["YOLOV5_CLASS_NAMES"][4].as<std::string>(),
              video_cfg["YOLOV5_CLASS_NAMES"][5].as<std::string>(), video_cfg["YOLOV5_CLASS_NAMES"][6].as<std::string>(),
              video_cfg["YOLOV5_CLASS_NAMES"][7].as<std::string>(), video_cfg["YOLOV5_CLASS_NAMES"][8].as<std::string>(),
              video_cfg["YOLOV5_CLASS_NAMES"][9].as<std::string>(), video_cfg["YOLOV5_CLASS_NAMES"][10].as<std::string>(),
              video_cfg["YOLOV5_CLASS_NAMES"][11].as<std::string>(), video_cfg["YOLOV5_CLASS_NAMES"][12].as<std::string>(),
              video_cfg["YOLOV5_CLASS_NAMES"][13].as<std::string>(), video_cfg["YOLOV5_CLASS_NAMES"][14].as<std::string>(),
              video_cfg["YOLOV5_CLASS_NAMES"][15].as<std::string>(), video_cfg["YOLOV5_CLASS_NAMES"][16].as<std::string>(),
              video_cfg["YOLOV5_CLASS_NAMES"][17].as<std::string>(), video_cfg["YOLOV5_CLASS_NAMES"][18].as<std::string>()};
    
    // 读取各目标类型的置信度阈值
    for (const auto& value : video_cfg["YOLOV5_CONF_THRESHOLD"]) {
        conf_thresholds.push_back(value.as<float>());
        }

    _input_frame_W = video_cfg["COMPRESS_IMG_W"].as<int>();  
    _input_frame_H = video_cfg["COMPRESS_IMG_H"].as<int>(); 
    _project_frame_W = video_cfg["ORIGINAL_IMG_W"].as<int>();  
    _project_frame_H = video_cfg["ORIGINAL_IMG_H"].as<int>(); 
    _GroundHeight = video_cfg["GROUND_HEIGHT"].as<float>();                                  // 地面点高度

    use_warpersepctivematrix = video_cfg["USE_WARPERSEPCTIVEMATRIX"].as<bool>();            // 使用单应性矩阵
    use_tilemap = video_cfg["USE_TILEMAP"].as<bool>();                                      // 使用瓦片图
    use_camerarelect = video_cfg["USE_CAMERARELECT"].as<bool>();                            // 使用相机映射

    Camera_num = p_pAlgParam->m_nCameraNum;             // 初始化相机个数
    VideoClass = p_pAlgParam->m_vecVideoClass;          // 初始化视频类别

    //  2. 加载单应性矩阵
    if(use_warpersepctivematrix){
        loadWarpMatrix(p_pAlgParam);
    }
    // 3. 瓦片图信息读取
    else if (use_tilemap){
        // 获取主雷达经纬度
        if(p_pAlgParam->m_tLidarParam.vecLidarDev().size() > 0){
            lidar_Lon = p_pAlgParam->m_tLidarParam.vecLidarDev()[0].dLidarLon();
            lidar_lat = p_pAlgParam->m_tLidarParam.vecLidarDev()[0].dLidarLat();
            lidar_angle = p_pAlgParam->m_tLidarParam.vecLidarDev()[0].fLidarNorthAngle();
        }
        else{
            LOG(INFO) << "Main lidar missing latitude and longitude" ;
        }

        for (int i = 0; i < Camera_num; i++){
            auto tiles_path = p_pAlgParam->m_strRootPath + "/Configs/Alg/video/postprocess" + "/test_tiles" + std::to_string(i) + ".npy";
            std::cout << tiles_path << std::endl;
            xt::xarray<double> tiles = xt::load_npy<double>(tiles_path);
            tiles_map[i] = tiles;

            // 加载掩码图
            std::string filter_image_path = p_pAlgParam->m_strRootPath + "/Configs/Alg/video/postprocess/back_filter" + std::to_string(i) + ".png";
            load_filter_image(filter_image_path, i);
        }

        // 读取配置文件
        std::string xml_path = p_pAlgParam->m_strRootPath + "/Configs/Alg/video/postprocess" + "/CameraTileMap.xml";
        std::cout << "xml_path : " << xml_path << std::endl;
        read_videofusion_xml(xml_path.c_str());
    }
    //  4. 相机映射关系读取
    else if(use_camerarelect){
        // 加载映射索引表
        for (int i = 0; i < Camera_num; i++){
            auto tiles_path = p_pAlgParam->m_strRootPath + "/Configs/Alg/video/postprocess/pixel2lidar_channel" + std::to_string(i) + ".npy";
            xt::xarray<double> tiles = xt::load_npy<double>(tiles_path);
            tiles_map[i] = tiles;

            // 加载掩码图
            std::string filter_image_path = p_pAlgParam->m_strRootPath + "/Configs/Alg/video/postprocess/back_filter" + std::to_string(i) + ".png";
            load_filter_image(filter_image_path, i);
        }
    }
    // 5. 读取目标尺寸信息
    auto class_size_path = p_pAlgParam->m_strRootPath + "/Configs/Alg/video/postprocess" + "/class_size.csv";
    class_size = csv2Xtensor<float>(class_size_path, 12, 3);

    LOG(INFO)<< "CVPost::init  status :  finish!" <<std::endl;
}


void CVPost::loadWarpMatrix(CSelfAlgParam* p_pAlgParam){
    // 1. 读取透视变化矩阵
    xt::xarray<float> WarpMatrixConf;
    std::string warpMatrix_path = p_pAlgParam->m_strRootPath + "/Configs/Alg/video/postprocess" + "/warpMatrix_init.csv";
    WarpMatrixConf = csv2Xtensor<float>(warpMatrix_path, 3, 20);
    WarpMatrixConf = WarpMatrixConf.reshape({3, 4, 5});
    // 2. 加载图像投射变换孪生参数
    xt::xarray<float> corner_2d_sum = xt::view(WarpMatrixConf,xt::all(),xt::all(),xt::range(0,2));
    xt::xarray<float> corner_3d_sum = xt::view(WarpMatrixConf,xt::all(),xt::all(),xt::range(2,5));
    
    for (int i = 0; i < corner_2d_sum.shape(0); ++i){
        xt::xarray<float> warpMatrix;
        xt::xarray<float> corner_2d=xt::view(corner_2d_sum,i,xt::all(),xt::all());
        xt::xarray<float> corner_3d=xt::view(corner_3d_sum,i,xt::all(),xt::all());
        WarPersepctiveMatrix(corner_2d, corner_3d, warpMatrix);
        warpMatrix_vec.push_back(warpMatrix);                               
    }
}

void CVPost::read_videofusion_xml(const char* videofusion_xml_path)
{
    XMLDocument fusion_alg_doc;
    if (fusion_alg_doc.LoadFile(videofusion_xml_path) != tinyxml2::XML_SUCCESS) {
        LOG(INFO) << "************************* load %s failed!!!!!!!!!!!! : " << videofusion_xml_path;
        return;
    }
    XMLElement* config_ele = fusion_alg_doc.FirstChildElement("CONFIG");
    XMLElement* VideoFusionParamAll = config_ele->FirstChildElement("VideoFusionAlgParam");
    // const char* strFusionClass = VideoFusionParamAll->FirstChildElement("m_FusionClass")->GetText(); 
    // char* strClass = strtok(const_cast<char*>(strFusionClass),",");
    // while(strClass!=nullptr)
    // {
    //     std::string one_class = strClass;
    //     m_vecFusionClass.push_back(one_class);
    //     strClass = strtok(nullptr,",");
    // }
    XMLElement* VideoFusionParam0 = VideoFusionParamAll->FirstChildElement("m_VideoFusionAlgParam_0");
    while(VideoFusionParam0){
        // start************************** m_VideoFusionAlgParam_0 *****************************//
        const char* CameraIp = VideoFusionParam0->FirstChildElement("m_unCameraIp")->GetText();  
        // std::string m_strNpyPath = VideoFusionParam0->FirstChildElement("m_strNpyPath")->GetText();  

        if(!(CameraIp == nullptr || strlen(CameraIp) == 0)){
            std::string m_unCameraIp = CameraIp;

            std::vector<std::vector<double>> m_vecRmF;
            XMLElement* RmF_node = VideoFusionParam0->FirstChildElement("m_vecRmF");  
            XMLElement* RmF_node0 = RmF_node->FirstChildElement();
            while (RmF_node0){
                std::vector<double> m_vecRmF_inter;
                const char* StrRmF = RmF_node0->GetText();
                if(!(StrRmF == nullptr || strlen(StrRmF) == 0)){
                    char* RmF = strtok(const_cast<char*>(StrRmF),",");
                    while(RmF!=nullptr)
                    {
                        double tmp_value = atof(RmF);
                        m_vecRmF_inter.push_back(tmp_value);
                        RmF = strtok(nullptr,",");
                    }
                }
                m_vecRmF.push_back(m_vecRmF_inter);
                if (RmF_node0->NextSiblingElement()){
                    RmF_node0 = RmF_node0->NextSiblingElement();
                }
                else{
                    RmF_node0 = nullptr;
                }
            }

            std::vector<std::vector<double>> m_vec_x_range;
            XMLElement* x_range_node = VideoFusionParam0->FirstChildElement("m_vec_x_range");  
            XMLElement* x_range_node0 = x_range_node->FirstChildElement();
            while (x_range_node0){
                std::vector<double> x_range_inter;
                const char* StrXRange = x_range_node0->GetText();
                if(!(StrXRange == nullptr || strlen(StrXRange) == 0)){
                    char* XRange = strtok(const_cast<char*>(StrXRange),",");
                    while(XRange!=nullptr)
                    {
                        double tmp_value = atof(XRange);
                        x_range_inter.push_back(tmp_value);
                        XRange = strtok(nullptr,",");
                    }
                }
                m_vec_x_range.push_back(x_range_inter);
                if (x_range_node0->NextSiblingElement()){
                    x_range_node0 = x_range_node0->NextSiblingElement();
                }
                else{
                    x_range_node0 = nullptr;
                }
            }

            std::vector<std::vector<double>> m_vec_x_adjust;
            XMLElement* x_adjust_node = VideoFusionParam0->FirstChildElement("m_vec_x_adjust");  
            XMLElement* x_adjust_node0 = x_adjust_node->FirstChildElement();
            while (x_adjust_node0){
                std::vector<double> x_adjust_inter;
                const char* StrXAdjust = x_adjust_node0->GetText();
                if(!(StrXAdjust == nullptr || strlen(StrXAdjust) == 0)){
                    char* XAdjust = strtok(const_cast<char*>(StrXAdjust),",");
                    while(XAdjust!=nullptr)
                    {
                        double tmp_value = atof(XAdjust);
                        x_adjust_inter.push_back(tmp_value);
                        XAdjust = strtok(nullptr,",");
                    }
                }
                m_vec_x_adjust.push_back(x_adjust_inter);
                if (x_adjust_node0->NextSiblingElement()){
                    x_adjust_node0 = x_adjust_node0->NextSiblingElement();
                }
                else{
                    x_adjust_node0 = nullptr;
                }
            }

            std::vector<std::vector<double>> m_vec_y_range;
            XMLElement* y_range_node = VideoFusionParam0->FirstChildElement("m_vec_y_range");  
            XMLElement* y_range_node0 = y_range_node->FirstChildElement();
            while (y_range_node0){
                std::vector<double> y_range_inter;
                const char* StrYRange = y_range_node0->GetText();
                if(!(StrYRange == nullptr || strlen(StrYRange) == 0)){
                    char* YRange = strtok(const_cast<char*>(StrYRange),",");
                    while(YRange!=nullptr)
                    {
                        double tmp_value = atof(YRange);
                        y_range_inter.push_back(tmp_value);
                        YRange = strtok(nullptr,",");
                    }
                }
                m_vec_y_range.push_back(y_range_inter);
                if (y_range_node0->NextSiblingElement()){
                    y_range_node0 = y_range_node0->NextSiblingElement();
                }
                else{
                    y_range_node0 = nullptr;
                }
            }

            std::vector<std::vector<double>> m_vec_y_adjust;
            XMLElement* y_adjust_node = VideoFusionParam0->FirstChildElement("m_vec_y_adjust");  
            XMLElement* y_adjust_node0 = y_adjust_node->FirstChildElement();
            while (y_adjust_node0){
                std::vector<double> y_adjust_inter;
                const char* StrYAdjust = y_adjust_node0->GetText();
                if(!(StrYAdjust == nullptr || strlen(StrYAdjust) == 0)){
                    char* YAdjust = strtok(const_cast<char*>(StrYAdjust),",");
                    while(YAdjust!=nullptr)
                    {
                        double tmp_value = atof(YAdjust);
                        y_adjust_inter.push_back(tmp_value);
                        YAdjust = strtok(nullptr,",");
                    }
                }
                m_vec_y_adjust.push_back(y_adjust_inter);
                if (y_adjust_node0->NextSiblingElement()){
                    y_adjust_node0 = y_adjust_node0->NextSiblingElement();
                }
                else{
                    y_adjust_node0 = nullptr;
                }
            }

            // 结果放到map中
            m_mapRmF[std::stoi(m_unCameraIp)] = m_vecRmF;
            m_mapXrange[std::stoi(m_unCameraIp)] = m_vec_x_range;
            m_mapXadjust[std::stoi(m_unCameraIp)] = m_vec_x_adjust;
            m_mapYrange[std::stoi(m_unCameraIp)] = m_vec_y_range;
            m_mapYadjust[std::stoi(m_unCameraIp)] = m_vec_y_adjust;
            // end************************** m_VideoFusionAlgParam_0 *****************************//
        }
        if (VideoFusionParam0->NextSiblingElement()){
            VideoFusionParam0 = VideoFusionParam0->NextSiblingElement();
        }
        else{
            VideoFusionParam0 = nullptr;
        }
    }
    return; 
}


/*
* 功能：执行视频检测后处理操作，包括NMS、标准化检测框、结果处理和数据设置
*/
void CVPost::execute()
{   
    LOG(INFO)<< "CVPost execute start"<<std::endl;
    auto t_start = std::chrono::high_resolution_clock::now();
    algResult.vecFrameResult().clear(); 

    // 1. 输入数据获取
    float * buffer = static_cast<float *>(getCommonData()->cpu_buffer);                 // 获取推理结果
    if (!buffer){
        LOG(ERROR)<< "CVPost::execute incoming data is empty"<<std::endl;
        return;                         // 图像后处理输入数据为空时，直接跳过后处理
    }     

    // 2. 进行批量NMS
    int batch_size = static_cast<std::vector<cv::Mat>>(getCommonData()->img_batch).size();
    std::vector<std::vector<yolov5Detection>> res_batch;
    batch_nms(res_batch, buffer, batch_size, kOutputSize, conf_thresholds, kNmsThresh);

    // 3. 标准化检测目标
    standard_box(res_batch);            // 将检测结果反向推导值左上坐标 宽度高度 尺寸坐标


    // 4. 视频检测信息处理
    result_process(res_batch);

    // 5. 设置输出数据
    for (int i = 0; i < algResult.vecFrameResult().size(); i++){
        LOG(INFO) << "Cemera " << std::to_string(i) << " Detecte Nums: " << algResult.vecFrameResult()[i].vecObjectResult().size() << std::endl;
    }
    
    // 6. 后处理模块输出
    m_CommonData->videoAlgResult = algResult;
    auto t_end = std::chrono::high_resolution_clock::now();
    auto latency_sp = (std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_end - t_start).count())/1000;
    LOG(INFO)<< "CVPost::execute status : finish!  >>> All Time :"<< latency_sp << " ms" <<std::endl;
}

/*
* 功能：将检测框标准化到原始图像尺寸
* 参数:
*   input: 引用，包含检测结果的二维向量
*/
void CVPost::standard_box(std::vector<std::vector<yolov5Detection>> &input)
{   
    float l, r, t, b;
    float r_w = _input_frame_W / (_input_frame_W * 1.0);
    float r_h = _input_frame_H / (_input_frame_H * 1.0);
    for (int i = 0; i < input.size(); ++i)          // 遍历每张图的检测结果
    {
        for (int j = 0; j < input[i].size(); ++j)   // 遍历每个检测目标
        {
            if (r_h > r_w)
            {
                l = input[i][j].bbox[0] - input[i][j].bbox[2] / 2.f;
                r = input[i][j].bbox[0] + input[i][j].bbox[2] / 2.f;
                t = input[i][j].bbox[1] - input[i][j].bbox[3] / 2.f - (_input_frame_H - r_w * _input_frame_H) / 2;
                b = input[i][j].bbox[1] + input[i][j].bbox[3] / 2.f - (_input_frame_H - r_w * _input_frame_H) / 2;
                l = l / r_w;
                r = r / r_w;
                t = t / r_w;
                b = b / r_w;
            }
            else
            {
                l = input[i][j].bbox[0] - input[i][j].bbox[2] / 2.f - (_input_frame_W- r_h * _input_frame_W) / 2;
                r = input[i][j].bbox[0] + input[i][j].bbox[2] / 2.f - (_input_frame_W - r_h * _input_frame_W) / 2;
                t = input[i][j].bbox[1] - input[i][j].bbox[3] / 2.f;
                b = input[i][j].bbox[1] + input[i][j].bbox[3] / 2.f;
                l = l / r_w;
                r = r / r_w;
                t = t / r_w;
                b = b / r_w;
            }
            input[i][j].rect = {l, t, r, b, r - l, b - t};            // left_top_x, left_top_y, right_bottom_x, right_bottom_y, w, h(640, 640)

        }
    }
}

/*
* 功能：处理检测结果，设置输出数据
* 参数:
*   res_batch: 引用，包含批量检测结果的二维向量
*/
void CVPost::result_process(std::vector<std::vector<yolov5Detection>> &res_batch){
    CVideoSrcDataTimematch video_Srcdata = getCommonData()->img_SrcData;
    // 遍历感知结果，设置输出数据
    for (int i = 0; i < res_batch.size(); i++){
        // 定义多个目标
        std::vector<CObjectResult> objectsResult;
        // 定义单帧目标
        CFrameResult frameResult;
        for (int j = 0; j < res_batch[i].size(); j++){
            // 获取检测目标
            if (res_batch[i][j].rect.width > _input_frame_W || res_batch[i][j].rect.width < 0 || res_batch[i][j].rect.height > _input_frame_H || res_batch[i][j].rect.height < 0 || res_batch[i][j].rect.left_top_x < 0 || res_batch[i][j].rect.left_top_y < 0){
                continue;
            } 
            // 定义单个目标
            CObjectResult object; 
            object.fTopLeftX(res_batch[i][j].rect.left_top_x);              // 640
            object.fTopLeftY(res_batch[i][j].rect.left_top_y);
            object.fBottomRightX(res_batch[i][j].rect.right_bottom_x );
            object.fBottomRightY(res_batch[i][j].rect.right_bottom_y);
            object.fPcConfidence(res_batch[i][j].conf * 100);
            object.strClass(VideoClass[int(res_batch[i][j].class_id)]);
            // 获取目标3维尺寸信息
            int class_id = int(res_batch[i][j].class_id);
            object.usWidth(class_size(class_id, 0) * 100);
            object.usLength(class_size(class_id, 1) * 100);
            object.usHeight(class_size(class_id, 2) * 100);
            // 使用瓦片图 or 单应性获取目标3D信息 or 相机映射
            if(!get_xyz(object, video_Srcdata.vecVideoSrcData()[i].ucCameraId(), class_id)) continue ;
            object.sSpeed(1 * 100);
            object.sCourseAngle(100 * PI / 180 * 100);
            object.ucSource(int(1));
            // 目标信息添加到 objectsResult
            objectsResult.push_back(object);
            // std::cout << i << " : " << object.sXCoord() << ", "<< object.sYCoord()<< ", " << object.sZCoord()<< ", " << object.usWidth()<< ", " << object.usLength()<< ", " << object.usHeight()<< ", " << std::endl;
        }
    // 多个目标信息添加到 frameResult
    frameResult.vecObjectResult(objectsResult);

    // 单帧目标信息添加到 algResult
    algResult.vecFrameResult().push_back(frameResult);
    }
}


//图像孪生加载      // 单应性矩阵计算  （图像 --> 激光雷达）
void CVPost::WarPersepctiveMatrix(xt::xarray<float> &corner_2d, xt::xarray<float> &corner_3d,
                                            xt::xarray<float> &warpMatrix) {
    //4*2 2d_corner, 4*3 3d_corner
    int nums = corner_2d.shape(0);
    xt::xarray<float> A = xt::zeros<float> ({2 * nums, 8});
    xt::xarray<float> B = xt::zeros<float> ({2 * nums, 1});
    xt::xarray<float> corner3d_reshape = xt::zeros<float> ({nums, 2});

    for (int i = 0; i < nums; ++i) {
        corner3d_reshape(i, 0) = corner_3d(i, 0) / corner_3d(i, 2);
        corner3d_reshape(i, 1) = corner_3d(i, 1) / corner_3d(i, 2);
        xt::xarray<float> A_i = xt::view(corner_2d, i, xt::all());
        xt::xarray<float> B_i = xt::view(corner3d_reshape, i, xt::all());
        xt::view(A, 2 * i, xt::all()) = xt::xarray<float> ({A_i(0), A_i(1), 1,
                                                            0, 0, 0,
                                                            -A_i(0)*B_i(0), -A_i(1)*B_i(0)});
        xt::view(B, 2 * i, xt::all()) = B_i(0);
        xt::view(A, 2 * i + 1, xt::all()) = xt::xarray<float> ({0, 0, 0,
                                                                A_i(0), A_i(1), 1,
                                                                -A_i(0)*B_i(1), -A_i(1)*B_i(1)});
        xt::view(B, 2 * i + 1,xt::all()) = B_i(1);
    }

    warpMatrix = xt::linalg::solve(A,B);            // 进行方程组求解 AX=B

    xt::xarray<float> c3_temp = xt::xarray<float>({1});
    c3_temp = c3_temp.reshape({1,1});
    warpMatrix = xt::concatenate(xt::xtuple(warpMatrix, c3_temp), 0);
    warpMatrix = warpMatrix.reshape({3,3});
}



bool CVPost::get_xyz(CObjectResult &object, int cameraID , int class_id){
    // 3.根据不同相机映射关系计算目标
    if (use_tilemap){
        std::cout << "use_tilemap" << std::endl;
        bool flag = video_to_wa(object, cameraID, class_id);
        return flag;
    }
    else {
        // 1.计算检测结果在原始图像上的中心点坐标
        int center_x = (int)(0.5 * (object.fTopLeftX() + object.fBottomRightX()) * _project_frame_W / _input_frame_W);
        int center_y = (int)(0.5 * (object.fTopLeftY() + object.fBottomRightY()) * _project_frame_H / _input_frame_H); 

        // 2.动态选取目标质心
        auto all_object_h = (int)((object.fBottomRightY() - object.fTopLeftY()) * _project_frame_H / _input_frame_H);
        auto all_object_w = (int)((object.fBottomRightX() - object.fTopLeftX()) * _project_frame_W / _input_frame_W);
        auto all_center_x = (int)center_x;
        auto all_center_y = (int)(object.fBottomRightY() * _project_frame_H / _input_frame_H);
        auto reflect_x_1 = (int)all_center_x;
        auto reflect_y_1 = (int)all_center_y;
        auto all_h_ratio = static_cast<float>(all_center_y / _project_frame_H * 0.25);
        auto all_w_ratio = static_cast<float>(all_center_x / _project_frame_W * 0.25);
        auto reflect_x_2 = static_cast<int>(all_center_x + all_w_ratio * all_object_w);
        auto reflect_y_2 = static_cast<int>(all_center_y - all_h_ratio * all_object_h);

        int new_center_x = 0, new_center_y = 0;
        if ( (0 <= all_center_y && all_center_y < _project_frame_H) && (0 <= all_center_x && all_center_x < _project_frame_W)){
            if ((0 <= reflect_y_1 && reflect_y_1 < _project_frame_H) && (0 <= reflect_x_1 && reflect_x_1 < _project_frame_W))
            {
                new_center_x = reflect_x_1;
                new_center_y = reflect_y_1;
            }else if((0 <= reflect_y_2 && reflect_y_2 < _project_frame_H) && (0 <= reflect_x_2 && reflect_x_2 < _project_frame_W))
            {
                new_center_x = reflect_x_2;
                new_center_y = reflect_y_2;
            }
            else{
                return false;
            }
            // 若新的质心在掩码图中像素点不为白 直接跳过
            if(m_mapfilter_image_flag[cameraID].at<int>(new_center_y, new_center_x) != 0){
                return false ;
            }
        }
        else return false;

        if(use_camerarelect){
            //std::cout << "new_center_x" << new_center_x << ", " << _project_frame_H << " ,"<< all_center_x << std::endl;
            object.sXCoord(tiles_map[cameraID](new_center_x, new_center_y, 0) * 100);                // X  : cm
            object.sYCoord(tiles_map[cameraID](new_center_x, new_center_y, 1) * 100);              // Y   : cm
            object.sZCoord(tiles_map[cameraID](new_center_x, new_center_y, 2) * 100);
            return true;
        }
        else if(use_warpersepctivematrix){
            xt::xarray<int> bottom_point = xt::cast<int>(xt::xarray<int>({center_x, center_y, 1}));
            xt::xarray<float> dst = xt::linalg::dot(warpMatrix_vec[cameraID], bottom_point);                            // 计算相机投射到雷达的坐标
            object.sXCoord(dst(0,0) / dst(2,0) * _GroundHeight);
            object.sYCoord(dst(1,0) / dst(2,0) * _GroundHeight);
            object.sZCoord(dst(2,0) / dst(2,0) * _GroundHeight);
            return true;
        }

    }
}

bool CVPost::video_to_wa(CObjectResult &object, int cameraID, int class_id)
{
    int center_x = (int)(0.5 * (object.fTopLeftX() + object.fBottomRightX()) * _project_frame_W / _input_frame_W);
    int center_y = (int)(0.5 * (object.fTopLeftY() + object.fBottomRightY()) * _project_frame_H / _input_frame_H); 
    std::cout << center_x << ", y:" << center_y << std::endl;
    // Y轴矫正
    std::cout << "Y : " << std::endl;
    for (int ii = 0; ii < m_mapYrange[cameraID].size(); ++ii)
    {
        bool IsValid = true;
        
        //判断是否在矫正区域内
        for (int j = 0; j < int (m_mapYrange[cameraID][ii].size() / 4); ++j)
        {
            double a = m_mapYrange[cameraID][ii][j * 4 + 0];
            double b = m_mapYrange[cameraID][ii][j * 4 + 1];
            double c = m_mapYrange[cameraID][ii][j * 4 + 2];
            double s = m_mapYrange[cameraID][ii][j * 4 + 3];
            bool flag = (a * center_x + b * center_y + c) * s > 0;
            IsValid &= flag; 
        }
        
        //小车矫正y
        if (IsValid && m_mapYadjust[cameraID][ii].size() != 0 && m_mapYrange[cameraID][ii].size() != 0 && !(class_id == 4 || class_id == 5 || class_id == 6 || class_id == 16))
        {
            if (center_y < m_mapYadjust[cameraID][ii][1] && center_y > m_mapYadjust[cameraID][ii][0])
            {    
                double adjust_y = (m_mapYadjust[cameraID][ii][3] - m_mapYadjust[cameraID][ii][2]) * 
                                ((center_y - m_mapYadjust[cameraID][ii][0]) / (m_mapYadjust[cameraID][ii][1] - m_mapYadjust[cameraID][ii][0]));
                center_y = int(object.fTopLeftY() + ((object.fBottomRightY() - object.fTopLeftY()) * (m_mapYadjust[cameraID][ii][2] + adjust_y)));
            }
        }
        //大车矫正y
        if (IsValid && m_mapYadjust[cameraID][ii].size() != 0 && m_mapYrange[cameraID][ii].size() != 0 && (class_id == 4 || class_id == 5 || class_id == 6 || class_id == 16))
        {            
            if (center_y < m_mapYadjust[cameraID][ii][1] && center_y > m_mapYadjust[cameraID][ii][0])
            {   
                double adjust_y = (m_mapYadjust[cameraID][ii][7] - m_mapYadjust[cameraID][ii][6]) * 
                                ((center_y - m_mapYadjust[cameraID][ii][4]) / (m_mapYadjust[cameraID][ii][5] - m_mapYadjust[cameraID][ii][4]));
                center_y = int(object.fTopLeftY() + ((object.fBottomRightY() - object.fTopLeftY()) * (m_mapYadjust[cameraID][ii][6] + adjust_y)));
            }
        }
    }
    std::cout << "X : " << std::endl;
    // X轴矫正
    for (int ii = 0; ii < m_mapXrange[cameraID].size(); ++ii)
    {
        bool IsValid = true;
        
        //判断是否在矫正区域内
        for (int j = 0; j < int (m_mapXrange[cameraID][ii].size() / 4); ++j)
        {
            double a = m_mapXrange[cameraID][ii][j * 4 + 0];
            double b = m_mapXrange[cameraID][ii][j * 4 + 1];
            double c = m_mapXrange[cameraID][ii][j * 4 + 2];
            double s = m_mapXrange[cameraID][ii][j * 4 + 3];
            bool flag = (a * center_x + b * center_y + c) * s > 0;
            IsValid &= flag; 
        }
        
        //小车矫正x
        if (IsValid && m_mapXadjust[cameraID][ii].size() != 0 && m_mapXrange[cameraID][ii].size() != 0 && !(class_id == 4 || class_id == 5 || class_id == 6 || class_id == 16))
        {
            if (center_y < m_mapXadjust[cameraID][ii][1] && center_y > m_mapXadjust[cameraID][ii][0])
            {    
                double adjust_y = (m_mapXadjust[cameraID][ii][3] - m_mapXadjust[cameraID][ii][2]) * 
                                ((center_y - m_mapXadjust[cameraID][ii][0]) / (m_mapXadjust[cameraID][ii][1] - m_mapXadjust[cameraID][ii][0]));
                center_x = int(object.fTopLeftX() + ((object.fBottomRightX() - object.fTopLeftX()) * (m_mapXadjust[cameraID][ii][2] + adjust_y)));
            }
        }
        //大车矫正x
        if (IsValid && m_mapXadjust[cameraID][ii].size() != 0 && m_mapXrange[cameraID][ii].size() != 0 && (class_id == 4 || class_id == 5 || class_id == 6 || class_id == 16))
        {            
            if (center_y < m_mapXadjust[cameraID][ii][1] && center_y > m_mapXadjust[cameraID][ii][0])
            {   
                double adjust_y = (m_mapXadjust[cameraID][ii][7] - m_mapXadjust[cameraID][ii][6]) * 
                                ((center_y - m_mapXadjust[cameraID][ii][4]) / (m_mapXadjust[cameraID][ii][5] - m_mapXadjust[cameraID][ii][4]));
                center_x = int(object.fTopLeftX() + ((object.fBottomRightX() - object.fTopLeftX()) * (m_mapXadjust[cameraID][ii][6] + adjust_y)));
            }
        }
    }

    // 动态选取质心时，若超出最大边界时，使用最大宽度和高度进行填充
    if (center_x > _project_frame_W - 1) 
    {
        center_x = _project_frame_W - 1;
    }
    if (center_y > _project_frame_H - 1)
    {
        center_y = _project_frame_H - 1;
    }
    
    // 根据瓦片图获取目标的经纬度
    std::cout << "center_x:  " << center_x << "center_y : " << center_y << std::endl;
    double lon = tiles_map[cameraID](center_x, center_y, 2);
    double lat = tiles_map[cameraID](center_x, center_y, 3);
    // std::cout << "lon:  " << lon << "lat : " << lat << std::endl; 
    // std::cout << "m_mapfilter_image_flag:  " << m_mapfilter_image_flag[cameraID].at<int>(center_x, center_y) << std::endl; 
    if(m_mapfilter_image_flag[cameraID].at<int>(center_y, center_x) != 0) return false ;       // 若目标不在感兴趣区域，直接跳出该操作
    // 更新目标XYZ
    std::cout << " Update XYZ : " << cameraID << ", " << lon << ", "<< lat << std::endl;
    if(int(lon) && int(lat))
    {   
        // 根据m_mapRmF计算是否丢弃该目标
        bool IsValid = true;
        for (int rmfAreaNum = 0; rmfAreaNum < m_mapRmF[cameraID].size(); ++rmfAreaNum) {
            for (int j = 0; j < int(m_mapRmF[cameraID][rmfAreaNum].size() / 4); ++j) {
                double a = m_mapRmF[cameraID][rmfAreaNum][j * 4 + 0];
                double b = m_mapRmF[cameraID][rmfAreaNum][j * 4 + 1];
                double c = m_mapRmF[cameraID][rmfAreaNum][j * 4 + 2];
                double s = m_mapRmF[cameraID][rmfAreaNum][j * 4 + 3];
                bool flag = (a * lon + b * lat + c) * s > 0;
                if (!flag) {
                    IsValid = false;
                    continue;
                }
            }
        }
        // std::cout << "m_mapRmF : " << std::endl;
        // 目标被保留
        if (IsValid) {      
            float object_X = 0, object_Y = 0;
            // std::cout << "BLH_To_XYZ" << std::endl;
            BLH_To_XYZ(lon, lat, lidar_Lon, lidar_lat, lidar_angle, object_X, object_Y);        // 根据经纬度和和主雷达经纬度及北向夹角来更新目标XY
            object.sXCoord(object_X * 100);         // X(cm)
            object.sYCoord(object_Y * 100);         // Y(cm)
            object.sZCoord(_GroundHeight * 100);    // 高度(cm)
            object.dLon(lon);                       // 经度
            object.dLat(lat);                       // 纬度
            return true;
        }
        // std::cout << "sucess: " << std::endl;
    }
    std::cout << "over : " << std::endl;
    return false;
}

void CVPost::load_filter_image(std::string file_path, int cameraId)
{   
    cv::Mat imag = cv::imread(file_path, cv::IMREAD_COLOR);
	if (imag.empty()){
		std::cout << "can't open the image:"<< file_path << std::endl;
		// return -1;
	}
	cv::Size size = imag.size();
	int width = size.width;
	int height = size.height;
	
	// auto &filter_image_flag = *_filter_image_flag;
	cv::Mat m_filter_image_flag = cv::Mat::ones(height, width, CV_32S);
	for(int i = 0; i<height; i++){
		for(int j = 0; j<width; j++){
			cv::Vec3b pixel = imag.at<cv::Vec3b>(i, j);
			if(pixel[0] == 255 && pixel[1] == 255 && pixel[2] == 255){
				m_filter_image_flag.at<int>(i, j) = 0;
			}
		}
	}
    m_mapfilter_image_flag[cameraId] = m_filter_image_flag;
}
