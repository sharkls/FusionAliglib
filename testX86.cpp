#include <stdio.h>
#include <chrono>
#include <assert.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include "Thirdpart/av_opencv/opencv2/opencv.hpp"
#include "Thirdpart/av_opencv/opencv2/core/mat.hpp"
#include "Thirdpart/av_opencv/opencv2/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "xtensor/xnpy.hpp"
#include "xtensor/xcsv.hpp"
#include "xtensor/xarray.hpp"
#include "xtensor/xstrided_view.hpp"
#include "xtensor-blas/xlinalg.hpp"
#include "yaml-cpp/yaml.h"
#include "tinyxml2.h"
#include "json.hpp"
#include <thread>
#include "CSelfAlgParam.h"
#include "GlobalContext.h"
#include "ExportPcAlgLib.h"
#include "CPcSrcData.h"
#include "ExportVideoAlgLib.h"
#include "CVideoSrcData.h"
#include "ExportFusionAlgLib.h"
#include "ExportTestAlgLib.h"
#include "ExportTestAlgLib.h"
#include <experimental/filesystem>

using namespace tinyxml2;
using namespace std;
namespace fs = std::experimental::filesystem;

/*相机内外参   此处为qlg_192.168.1.133ip的相机参数*/
const float camera_param_in[4][9]     = {{1152.4, 0, 959.409, 0, 1147.4, 550.065, 0, 0, 1},
                                         {1147.9, 0, 951.689, 0, 1143.4, 566.33, 0, 0, 1},
                                         {1152.4, 0, 959.409, 0, 1147.4, 550.065,       0, 0, 1},
                                         {1152.4, 0, 959.409, 0, 1147.4, 550.065,       0, 0, 1}};
const float camera_param_rotate[4][3] = {{1.85233, 0.624684,  -0.484852},
                                         {2.02309, 0.128325,  -0.105201},
                                         {1.85233,   0.624684,   -0.484852},
                                         {1.85233,   0.624684,   -0.484852}};
const float camera_param_trans[4][3]  = {{234.929,   151.025, -0.075},
                                         {95.262,    164.969,  15.345},
                                         {234.929,            151.025,            -0.075},
                                         {234.929,            151.025,            -0.075}};
const float camera_param_dis[4][5]    = {{-0.3341, 0.1085, 0.000413,    0.00070638, 0},
                                         {-0.3335, 0.109,  -0.00068918, 0.0013,     0},
                                         {-0.3341, 0.1085, 0.000413,    0.00070638,  0},
                                         {-0.3341, 0.1085, 0.000413,    0.00070638,  0}};

int timestamp = 0;
void save_fusion_result_to_csv(std::string path, CAlgResult outputAlgResult);

inline void file_to_string(vector<string> &record, const string& line, char delimiter)
{
    int linepos=0;
    char c;
    int linemax=line.length();
    string curstring;
    record.clear();
    while(linepos<linemax)
    {
        c = line[linepos];
        if(isdigit(c)||c=='.'||c=='-'){
            curstring+=c;
        }
        else if(c==delimiter&&curstring.size()){
            record.push_back(curstring);
            curstring="";
        }
        ++linepos;
    }
    if(curstring.size())
        record.push_back(curstring);
    return;
}

inline float string_to_float(string str)
{
    int i=0,len=str.length();
    float sum=0;
    while(i<len){
        if(str[i]=='.') break;
        sum=sum*10+str[i]-'0';
        ++i;
    }
    ++i;
    float t=1,d=1;
    while(i<len){
        d*=0.1;
        t=str[i]-'0';
        sum+=t*d;
        ++i;
    }
    return sum;
}

bool isFileExists_ifstream(string &name) {
    ifstream f(name.c_str());
    if (!f.good()) {
        // std::cout<< name << "不存在，请检查!" << std::endl;
        return false;
    }
    return true;
}

void read_pc_data(const std::string & data_path, CPcSrcData& l_stPcSrcData)
{
    l_stPcSrcData.vecPoints().clear();
    ifstream in(data_path);
    if (in.fail())
    {
        return;
    }
    vector<string> row;
    string line;
    while(getline(in, line)  && in.good() )
    {
        file_to_string(row, line, ',');  //把line里的单元格数字字符提取出来，“,”为单元格分隔符
        CPcPoint l_cPcPoint;
        l_cPcPoint.fPointX(atof(row[0].data()));
        l_cPcPoint.fPointY(atof(row[1].data()));
        l_cPcPoint.fPointZ(atof(row[2].data()));
        l_cPcPoint.fIntensity(atof(row[3].data()));
        l_stPcSrcData.vecPoints().push_back(l_cPcPoint);
    }
    in.close();
}

namespace ops{
    inline long getTimeStamp()
    {
        long t_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        return t_now;
    }
    inline void Trim(std::string &str) {
        str.erase(0, str.find_first_not_of(" \t\r\n"));
        str.erase(str.find_last_not_of(" \t\r\n") + 1);
    }
    template<class Type>
    Type stringToNum(const std::string &str) {
        std::istringstream iss(str);
        Type num;
        iss >> num;
        return num;
    }
    template<class Type>
    std::vector<Type> vStrSplit(const std::string &strSur, const char &cConChar) {
        std::vector<Type> output;
        std::string::size_type pos1, pos2;
        pos1 = 0;
        pos2 = strSur.find(cConChar, 0);
        int i = 0;
        while (std::string::npos != pos2) {
            Type key = stringToNum<Type>(strSur.substr(pos1, pos2 - pos1));
            output.push_back(key);
            pos1 = pos2 + 1;
            pos2 = strSur.find(cConChar, pos1);
        }
        output.push_back(stringToNum<Type>(strSur.substr(pos1, strSur.size())));
        return output;
    }
    template<typename T>
    xt::xarray<T> csvReader(const std::string &csv_path) {
        std::ifstream csv_reader;
        csv_reader.open(csv_path);
        std::string line;
        std::size_t k = 0;
        std::vector<T> out;
        while (getline(csv_reader, line)) {
            k++;
            ops::Trim(line);
            std::vector<T> key = ops::vStrSplit<T>(line, ',');
            out.insert(out.end(), key.begin(), key.end());
        }
        if (k == 0)
        {
            xt::xarray<T> res = {0};
            return res;
            //            throw("k===0");
        }
        Update_tracks :
        std::vector<std::size_t> shape = {k, std::size_t(out.size() / k)};
        xt::xarray<T> res = xt::adapt(out, shape);
        return res;
    }
};

std::vector<std::string> getFilesList(const std::string &dirpath)
{
    DIR *dir = opendir(dirpath.c_str());
    if (dir == NULL)
    {
        std::cout << "Open dir error: Doesn't exist! \n";
    }
    std::vector<std::string> all_path;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL)
    {
        if (entry->d_type == DT_DIR)
        { // it is a dir
            if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            {
                continue;
            }
            std::cout << ">>>> It as a dir path!\n";
            std::string dir_new = entry->d_name;
            std::vector<std::string> temp_path = getFilesList(dir_new);
            all_path.insert(all_path.end(), temp_path.begin(), temp_path.end());
        }
        else
        {
            std::string name = entry->d_name;

            all_path.push_back(name);
        }
    }
    closedir(dir);
    std::sort(all_path.begin(), all_path.end());
    return all_path;
}

// // 视频回调函数（测试视频算法使用）
void test_vb(const CAlgResult& ms, void* hd)
{

    std::cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&& test_vb &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& \n";
    std::ofstream file;
    file.open(std::string("/data/AlgLib/Output/VideoResult/2/back/") + "Frame_" + std::to_string(ms.vecFrameResult()[0].unFrameId()) + ".csv");

    // 检查文件是否成功打开
    if (!file.is_open()) {
        std::cout << "无法打开文件" << std::endl;
    }
    else
    {   // 设置写出的检测结果的单位大小
        float scale_factor_width = 1920.0 / 640.0;
        float scale_factor_height = 1080.0 / 640.0;
        // 若感知结果的目标数大于0
        if (ms.vecFrameResult()[0].vecObjectResult().size() > 0){
            std::cout << "video 0 output object nums: "<< ms.vecFrameResult()[0].vecObjectResult().size() << std::endl;
            for (int i = 0; i < ms.vecFrameResult()[0].vecObjectResult().size(); i++){
                float x = ((ms.vecFrameResult()[0].vecObjectResult()[i].fTopLeftX() + ms.vecFrameResult()[0].vecObjectResult()[i].fBottomRightX()) / 2)  * scale_factor_width;
                float y = ((ms.vecFrameResult()[0].vecObjectResult()[i].fTopLeftY() + ms.vecFrameResult()[0].vecObjectResult()[i].fBottomRightY()) / 2) * scale_factor_height;
                float w = (ms.vecFrameResult()[0].vecObjectResult()[i].fBottomRightX() - ms.vecFrameResult()[0].vecObjectResult()[i].fTopLeftX()) * scale_factor_width;
                float h = (ms.vecFrameResult()[0].vecObjectResult()[i].fBottomRightY() - ms.vecFrameResult()[0].vecObjectResult()[i].fTopLeftY()) * scale_factor_height;
                file << " ,"<< ms.vecFrameResult()[0].vecObjectResult()[i].strClass()  << ","<< x << "," << y << ", , , , , " << w << "," << h<< "\n";
            }
        }
    }
    file.close();
}

void testVideo()
{
    std::string img_path = "/share/Code/FusionAliglib/data/video_data/";
    int CamNUm = 4;
    //算法接口调用流程基本如下：
    IVideoAlg* l_pObj = CreateVideoAlgObj("/share/Code/FusionAliglib/Output");
    //准备算法参数
    CSelfAlgParam *l_stVideoAlgParam = new CSelfAlgParam();
    l_stVideoAlgParam->m_strRootPath = "/share/Code/FusionAliglib/Output";
    l_stVideoAlgParam->m_strVideoCfgPath = "/Configs/Alg/video/video.yaml";
    l_stVideoAlgParam->m_nCameraNum = CamNUm;
    CCameraParam cameraParam;
    for (int i = 0; i < CamNUm; ++i)
    {   std::cout << l_stVideoAlgParam->m_tCameraParam.vecCameraDev().size() << std::endl;
        // l_stVideoAlgParam->m_tCameraParam.vecCameraDev()[i].usCameraId(i);
        std::vector<CCameraDev> vecCameraDev = cameraParam.vecCameraDev(); // 获取当前的m_vecCameraDev
        CCameraDev newCameraDev;
        newCameraDev.usCameraId(i); // 设置新的CCameraDev对象的属性
        vecCameraDev.push_back(newCameraDev); // 添加新的CCameraDev对象到向量中
        cameraParam.vecCameraDev(vecCameraDev); // 设置m_vecCameraDev的值
    }
    l_stVideoAlgParam->m_tCameraParam = cameraParam;

    // 设置主雷达经纬度
    CLidarDev Lidar1;

    /* 雷达参数 qlg*/
    Lidar1.dLidarLon(116.2915824224771);
    Lidar1.dLidarLat(40.0533201647222);
    Lidar1.fLidarNorthAngle(-99.791);
    l_stVideoAlgParam->m_tLidarParam.vecLidarDev().push_back(Lidar1);


    // 初始化算法接口对象
    l_pObj->InitAlgorithm(l_stVideoAlgParam,  test_vb, nullptr);

    // //准备视频数据。。。
    CVideoSrcDataTimematch l_pVideoSrcData;
    for (int i = 0; i < 600; i++)
    {
        for (int j = 0; j < CamNUm; ++j)
        {
            cv::Mat img_ori = cv::imread(img_path + std::to_string(i) + ".png");
            //  cv::Mat img_ori = cv::imread(img_path + "frame_" + std::to_string(i) + ".png");
            std::cout << "<<<<img_ori.size() : " << img_path + std::to_string(i) + ".png"  << std::endl;
            if (img_ori.empty()) {
                std::cerr << "Error: img_ori is empty. Failed to load image." << std::endl;
                continue; // 跳过当前迭代
            }
            cv::Size targetSize(640, 640);
            cv::Mat img;
            cv::resize(img_ori, img, targetSize, 0, 0, cv::INTER_LINEAR);

            // 直接使用img.data初始化vector
            std::vector<uint8_t> l_MatBuff(img.data, img.data + img.total() * img.elemSize());

            CVideoSrcData l_stVideoData;
            l_stVideoData.unFrameId(i);
            l_stVideoData.ucCameraId(j);
            std::map<uint8_t, uint64_t> timeStampMap;
            timeStampMap[0] = i;
            timeStampMap[1] = i;
            l_stVideoData.mapTimeStamp(timeStampMap);
            l_stVideoData.usBmpLength(640);
            l_stVideoData.usBmpWidth(640);
            l_stVideoData.vecImageBuf(l_MatBuff);
            l_stVideoData.unBmpBytes(l_MatBuff.size());

            // 获取 m_vecVideoSrcData，添加元素，然后重新设置
            std::vector<CVideoSrcData> vecVideoSrcData = l_pVideoSrcData.vecVideoSrcData();
            vecVideoSrcData.push_back(l_stVideoData);
            l_pVideoSrcData.vecVideoSrcData(vecVideoSrcData);
        }
        std::cout << "RunAlgorithm data start!:"<< i << std::endl;
        l_pObj->RunAlgorithm(&l_pVideoSrcData);
        usleep(1000*100);

        // 清除 m_vecVideoSrcData
        std::vector<CVideoSrcData> vecVideoSrcData = l_pVideoSrcData.vecVideoSrcData();
        vecVideoSrcData.clear();
        l_pVideoSrcData.vecVideoSrcData(vecVideoSrcData);
    }
    delete l_pObj;
}

// // 测试回调函数（测试算法使用）
void test_cb(const CAlgResult& ms, void* hd)
{
    std::cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&& test_vb &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& \n";
}

void test()
{
    std::string img_path = "/share/Code/FusionAliglib/data/video_data/";
    //算法接口调用流程基本如下：
    ITestAlg* l_pObj = CreateTestAlgObj("/share/Code/FusionAliglib/Output");
    //准备算法参数
    CSelfAlgParam *l_stTestAlgParam = new CSelfAlgParam();
    l_stTestAlgParam->m_strRootPath = "/share/Code/FusionAliglib/Output";
    l_stTestAlgParam->m_strTestCfgPath = "/Configs/Alg/test/test.yaml";

    // 初始化算法接口对象
    l_pObj->InitAlgorithm(l_stTestAlgParam,  test_cb, nullptr);

    // //准备视频数据。。。
    int CamNUm = 4;
    CVideoSrcDataTimematch l_pVideoSrcData;
    for (int i = 0; i < 600; i++)
    {
        for (int j = 0; j < CamNUm; ++j)
        {
            cv::Mat img_ori = cv::imread(img_path + std::to_string(i) + ".png");
            //  cv::Mat img_ori = cv::imread(img_path + "frame_" + std::to_string(i) + ".png");
            std::cout << "<<<<img_ori.size() : " << img_path + std::to_string(i) + ".png"  << std::endl;
            if (img_ori.empty()) {
                std::cerr << "Error: img_ori is empty. Failed to load image." << std::endl;
                continue; // 跳过当前迭代
            }
            cv::Size targetSize(640, 640);
            cv::Mat img;
            cv::resize(img_ori, img, targetSize, 0, 0, cv::INTER_LINEAR);

            // 直接使用img.data初始化vector
            std::vector<uint8_t> l_MatBuff(img.data, img.data + img.total() * img.elemSize());

            CVideoSrcData l_stVideoData;
            l_stVideoData.unFrameId(i);
            l_stVideoData.ucCameraId(j);
            std::map<uint8_t, uint64_t> timeStampMap;
            timeStampMap[0] = i;
            timeStampMap[1] = i;
            l_stVideoData.mapTimeStamp(timeStampMap);
            l_stVideoData.usBmpLength(640);
            l_stVideoData.usBmpWidth(640);
            l_stVideoData.vecImageBuf(l_MatBuff);
            l_stVideoData.unBmpBytes(l_MatBuff.size());

            // 获取 m_vecVideoSrcData，添加元素，然后重新设置
            std::vector<CVideoSrcData> vecVideoSrcData = l_pVideoSrcData.vecVideoSrcData();
            vecVideoSrcData.push_back(l_stVideoData);
            l_pVideoSrcData.vecVideoSrcData(vecVideoSrcData);
        }
        std::cout << "RunAlgorithm data start!:"<< i << std::endl;
        l_pObj->RunAlgorithm(&l_pVideoSrcData);
        usleep(1000*100);

        // 清除 m_vecVideoSrcData
        std::vector<CVideoSrcData> vecVideoSrcData = l_pVideoSrcData.vecVideoSrcData();
        vecVideoSrcData.clear();
        l_pVideoSrcData.vecVideoSrcData(vecVideoSrcData);
    }
    delete l_pObj;
}

// 融合算法回调函数（测试融合算法使用）
void fusionAlg_CallBack(const CAlgResult& ms, void* hd)
{
    std::cout << "********************** test_Fcb ****************** \n";
    std::string l_sOutPutPath = "/share/Code/FusionAliglib/Output/FusionResults/";
    save_fusion_result_to_csv(l_sOutPutPath, ms);
}

bool jsonload(nlohmann::json &json_data, std::string filepath)
{
    std::string::size_type name_pos = filepath.find_last_of('/') + 1;
    std::string filename = filepath.substr(name_pos, filepath.length() - name_pos);
    std::string file_extension = filepath.substr(filepath.find_last_of('.') + 1);
    std::string name = filename.substr(0, filename.rfind("."));
    if (file_extension.compare("json") != 0)
    {
        return false;
    }
    std::ifstream ifs(filepath, std::ios::in);
    if (!ifs.good())
    {
        std::cout << filepath << " No such Json File\n"
                  << std::endl;
        return false;
    }
    if (!(ifs >> (json_data[name])))
    {
        std::cout << filepath << " Error reading Json File\n"
                  << std::endl;
        return false;
    }
    return true;
}

void read_fusiondata(const std::string & data_path, int i, int cam_num, CAlgResult& l_FuTrSrcData, CSelfAlgParam *&l_stFuAlgParam)
{
    // Read Outline Video Data
    CFrameResult l_stFuSrcData_video;
    l_stFuSrcData_video.unFrameId(i);   // 帧号

    char frameNum[7];
    snprintf(frameNum, sizeof(frameNum), "%06d", i);

    // Iterate through camera detection data one by one
    for (int j = 0; j < cam_num; j++) 
    {
        auto video_path = data_path + "VideoBox_Channel" + std::to_string(j) + "/Frame_" + frameNum + ".npy" ;
        std::cout << "read video_path:" << video_path << std::endl;

        // Check if the file path exists
        char * cf = const_cast<char *>(video_path.c_str());
        struct stat sb;
        if (stat(cf, &sb) != 0)
        {
            std::cout<<video_path<<"file does not exist"<<endl;
            return;
        }

        // Load video data and push to CAlgResult instance
        xt::xarray<double> video_src_data = xt::load_npy<double>(video_path);
        CObjectResult videoSrcResult;
        l_stFuSrcData_video.tCameraSupplement().ucCameraId(j + 1);  // 相机id
        for (int k = 0; k < video_src_data.shape(0); k++)
        {
            CObjectResult m_videobox;
            m_videobox.sXCoord(video_src_data(k, 0));
            m_videobox.sYCoord(video_src_data(k, 1));
            m_videobox.usWidth(video_src_data(k, 2));
            m_videobox.usLength(video_src_data(k, 3));
            m_videobox.sCourseAngle(video_src_data(k, 4));
            m_videobox.sZCoord(video_src_data(k, 5));
            m_videobox.usHeight(video_src_data(k, 6));
            m_videobox.strClass(std::string(l_stFuAlgParam->m_fusion_parameter["fusion_param"]["m_strVideoClass"][video_src_data(k,7)]));
            m_videobox.sSpeed(video_src_data(k, 8));
            m_videobox.usTargetId(video_src_data(k, 9));
            m_videobox.fVideoConfidence(int(video_src_data(k, 10) * 100));
            m_videobox.fTopLeftX(video_src_data(k, 11)) ;       // 左上
            m_videobox.fTopLeftY(video_src_data(k, 12)) ;
            m_videobox.fBottomRightX(video_src_data(k, 13));    // 右下
            m_videobox.fBottomRightY(video_src_data(k, 14));
            m_videobox.ucSource(video_src_data(k, 15));
            // Add target information to CObjResult instance
            l_stFuSrcData_video.vecObjectResult().push_back(m_videobox);
        }
        // Add timestamp information and data type information
        l_stFuSrcData_video.eDataType() = DATA_TYPE_VIDEO_RESULT;
        l_stFuSrcData_video.mapTimeStamp()[TIMESTAMP_PCSRCINFO_SUB] = timestamp;

        // Push to CAlgResult instance
        l_FuTrSrcData.vecFrameResult().push_back(l_stFuSrcData_video);

        // Testing requirements
        CAlgResult PrintVideoFrame;
        PrintVideoFrame.vecFrameResult().push_back(l_stFuSrcData_video);
        // save_fusion_result_to_csv(data_path +"VideoBox_Channel" + std::to_string(j) +"_csv/", PrintVideoFrame);
    }

    //pointcloud
    string pc_path = data_path + "PcBox/Frame_" + frameNum  + ".npy";
    xt::xarray<double> pc_src_data =  xt::load_npy<double>(pc_path);

    CFrameResult l_stFuSrcData_pc;
    l_stFuSrcData_pc.unFrameId(i);
    for (int j = 0; j < pc_src_data.shape(0); j++)
    {
        CObjectResult pcbox;
        pcbox.sXCoord(int(pc_src_data(j, 0) * 100));        // 单位cm 
        pcbox.sYCoord(int(pc_src_data(j, 1) * 100));
        pcbox.sZCoord(int(pc_src_data(j, 2) * 100));
        pcbox.usWidth(int(pc_src_data(j, 3) * 100));
        pcbox.usLength(int(pc_src_data(j, 4) * 100));
        pcbox.usHeight(int(pc_src_data(j, 5) * 100));
        pcbox.sCourseAngle(pc_src_data(j, 6));
        pcbox.strClass(l_stFuAlgParam->m_fusion_parameter["fusion_param"]["m_strPcClass"][int(pc_src_data(j,7))]);
        pcbox.fPcConfidence(int(pc_src_data(j,8) * 100));
        pcbox.sSpeed(pc_src_data(j,9));
        pcbox.usTargetId(pc_src_data(j,10));
        pcbox.ucSource(pc_src_data(j,11));
        // 添加点云目标信息到CObjResult实例中
        l_stFuSrcData_pc.vecObjectResult().push_back(pcbox);
    }
    // 经纬度和雷达的北向夹角可以暂时不提供
    l_stFuSrcData_pc.tLidarSupplement().dLidarLon(l_stFuAlgParam->m_tLidarParam.vecLidarDev()[0].dLidarLon());
    l_stFuSrcData_pc.tLidarSupplement().dLidarLat(l_stFuAlgParam->m_tLidarParam.vecLidarDev()[0].dLidarLat());
    l_stFuSrcData_pc.tLidarSupplement().fLidarNorthAngle(l_stFuAlgParam->m_tLidarParam.vecLidarDev()[0].fLidarNorthAngle());
    l_stFuSrcData_pc.mapTimeStamp()[TIMESTAMP_PCSRCINFO_SUB] = timestamp;
    l_stFuSrcData_pc.eDataType(DATA_TYPE_PC_RESULT) ;
    // 添加当前点云帧所有目标信息到CAlgResult实例中
    l_FuTrSrcData.vecFrameResult().push_back(l_stFuSrcData_pc);

    // Testing requirements
    CAlgResult PrintPcFram; //
    PrintPcFram.vecFrameResult().push_back(l_stFuSrcData_pc);
    // save_fusion_result_to_csv(data_path + "PcBox_csv/", PrintPcFram);
}

void save_fusion_result_to_csv(std::string path, CAlgResult outputAlgResult){
    std::string rootPath = "/share/Code/FusionAliglib/";
    std::string filepath1 = rootPath + "Output/Configs/Alg/fusion/fusion_param.json";
    nlohmann::json temp_param1;
    bool isload = jsonload(temp_param1, filepath1);

    for(int i = 0 ; i < outputAlgResult.vecFrameResult().size(); i++){
        std::ofstream file;
        int fid = outputAlgResult.vecFrameResult()[i].unFrameId();
        string file_path = std::string(path + "Frame_" + std::to_string(fid) +".csv");
        fs::path pathObj = (file_path);
        if(!fs::exists(pathObj.parent_path()))
        {
            fs::create_directories(pathObj.parent_path());
        }
        file.open(file_path);
        // 检查文件是否成功打开
        if (!file.is_open()) {
            std::cout << "无法打开文件" << std::endl;
        }
        else
        {
            for (int j = 0; j < outputAlgResult.vecFrameResult()[i].vecObjectResult().size(); j++)
            {
                int ID = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].usTargetId();
                float X = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].sXCoord()/100.0;

                float y = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].sYCoord()/100.0 ;
                float z = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].sZCoord()/100.0 ;
                float w = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].usWidth()/100.0 ;
                float l = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].usLength()/100.0 ;
                float h = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].usHeight()/100.0 ;
                float s = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].sSpeed()/100.0 ;
                float a = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].sCourseAngle() ;
                std::string cls1 = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].strClass() ;
                float cls = 0;
                for(int i = 0; i < temp_param1["fusion_param"]["m_strFusionClass"].size(); i++)
                {
                    if(cls1 == temp_param1["fusion_param"]["m_strFusionClass"][int(i)])
                    {
                        cls = i;
                    }
                }

                int src = int(outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].ucSource()) ;
                double dlon1 = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].dLon() ;
                double dlat1 =outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].dLat() ;
                float Conf = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].fPcConfidence() ;
                int channel = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].usVideoInfo()[4];
                float ltx = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].fTopLeftX();
                float lty = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].fTopLeftY();
                float rbx = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].fBottomRightX();
                float rby = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].fBottomRightY();
                unsigned long long ti = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].ulFirstOccurTime();
                int lane = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].usLaneNum();

                file << ID <<","<< cls1<< ","<< X << "," << y << ", "<<z<<", "<< w<<","<< l <<" ,"<<h<<","<< s <<","<< a <<","<<src<<","<<Conf<<endl;
//                file << cls << "," << src << ","  << Conf << "," << X << "," << y << "," << z << "," << l << "," << w << "," << h << "," << a << "," << ltx << "," << lty << "," << rbx << "," << rby << "," << ID << "," << lane << "," << s << "," << ti << "," << channel <<endl;
            }
        }
        file.close();
    }
}

int testFusion()
{
    int cam_num = 4;
    std::string rootPath = "/share/Code/FusionAliglib/";
    IFusionAlg* l_pObj = CreateFusionAlgObj(rootPath + "Output");
    CSelfAlgParam *l_stFuAlgParam = new CSelfAlgParam();
    CLidarDev Lidar1;

    // 设置雷达参数 ：经纬度、北向夹角
    Lidar1.dLidarLon(120.626484706589);
    Lidar1.dLidarLat(31.4241149141);
    Lidar1.fLidarNorthAngle(92.22);
    l_stFuAlgParam->m_tLidarParam.vecLidarDev().push_back(Lidar1);

    // 设置相机参数
    l_stFuAlgParam->m_tCameraParam.vecCameraDev().resize(cam_num);
    // l_stFuAlgParam->m_tCameraParam.bUse485(false);
    l_stFuAlgParam->m_tCameraParam.unCameraCount(cam_num);
    for(int i = 0; i < cam_num; i++)
    {
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecInParameter().resize(9);            // 内参
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecRotateMatrix().resize(3);           // 旋转
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecTranslationMatrix().resize(3);      // 平移
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecDistMatrix().resize(5);             // 畸变系数

        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecInParameter().push_back(camera_param_in[i][0]);
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecInParameter().push_back(camera_param_in[i][1]);
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecInParameter().push_back(camera_param_in[i][2]);
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecInParameter().push_back(camera_param_in[i][3]);
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecInParameter().push_back(camera_param_in[i][4]);
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecInParameter().push_back(camera_param_in[i][5]);
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecInParameter().push_back(camera_param_in[i][6]);
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecInParameter().push_back(camera_param_in[i][7]);
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecInParameter().push_back(camera_param_in[i][8]);
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecRotateMatrix().push_back(camera_param_rotate[i][0]);
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecRotateMatrix().push_back(camera_param_rotate[i][1]);
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecRotateMatrix().push_back(camera_param_rotate[i][2]);
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecTranslationMatrix().push_back(camera_param_rotate[i][0]);
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecTranslationMatrix().push_back(camera_param_rotate[i][1]);
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecTranslationMatrix().push_back(camera_param_rotate[i][2]);
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecDistMatrix().push_back(camera_param_dis[i][0]);
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecDistMatrix().push_back(camera_param_dis[i][1]);
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecDistMatrix().push_back(camera_param_dis[i][2]);
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecDistMatrix().push_back(camera_param_dis[i][3]);
        l_stFuAlgParam->m_tCameraParam.vecCameraDev()[i].vecDistMatrix().push_back(camera_param_dis[i][4]);
    }
    
    // 加载配置文件参数
    std::string filepath1 = rootPath + "Output/Configs/Alg/fusion/fusion_param.json";
    nlohmann::json temp_param1;
    bool isload = jsonload(temp_param1, filepath1);
    l_stFuAlgParam->m_fusion_parameter = temp_param1;
    l_stFuAlgParam->m_strRootPath = "/data/AlgLib/Output";
    l_pObj->InitAlgorithm(l_stFuAlgParam, fusionAlg_CallBack, nullptr);

    // 加载测试数据
    for(int i = 278; i < 378; ++i) //i = the frame index
    {
        std::cout<<"Processing Frame: ("<<std::to_string(i)<<")  started."<<endl;
        CAlgResult *l_pSrcData = new CAlgResult();
        read_fusiondata(rootPath + "data/fusion_test_data/", i, cam_num, *l_pSrcData, l_stFuAlgParam);
        auto t_start = std::chrono::steady_clock::now();

        if (l_pSrcData->vecFrameResult().size() > 0)
        {   
            std::cout << "-----------------------------Enter FusionRunAlgorithm!" << std::endl;
            l_pObj->RunAlgorithm(l_pSrcData);
            timestamp = timestamp + 100;
            auto t_end = std::chrono::steady_clock::now();
            double latency = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        }
        usleep(1000*100);
        delete l_pSrcData;
    }

    delete l_pObj;
    delete l_stFuAlgParam;
    return 0;
}


// //测试算法接口调用流程
int main(int argc, char* argv[])
{
    // testVideo();
    // test();
    testFusion();
    return 0;
}