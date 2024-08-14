#include "Thirdpart/av_opencv/opencv2/core/mat.hpp"
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
#include "ExportEventAlgLib.h"
#include "ExportSceneAlgLib.h"
#include "ExportTriggerAlgLib.h"
#include "Export2K1VAlgLib.h"
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

// /*相机内外参   此处为上地西67，68，126ip的相机参数*/
// const float camera_param_in[3][9]     = {{1174.07467031771, 0, 975.559052741506, 0, 1167.92176860689, 572.720893252979, 0, 0, 1},
//                                          {1154.07467031771, 0, 975.559052741506, 0, 1207.92176860689, 572.720893252979, 0, 0, 1},
//                                          {1155.023180,      0, 967.178167,       0, 1149.531704,      541.645587,       0, 0, 1}};
// const float camera_param_rotate[3][3] = {{1.6303330135419367, -1.4019180977442631,  0.8723481431372321},
//                                          {1.3731985950150833,  1.5004915129867642  -1.1267008444124609},
//                                          {-2.4249080614788,   -1.2204040053466,    -2.31865241843}};
// const float camera_param_trans[3][3]  = {{680.48,             303.28000000000003, -250.43},
//                                          {-367.4599999999999, 393.5,              4.88},
//                                          {500.000,            -25.652,            70.477}};
// const float camera_param_dis[3][5]    = {{-0.318846891959627, 0.0902775208989156, 0.000282528081340622, 0.000800223746856249, 0},
//                                          {-0.318846891959627, 0.0902775208989156, 0.000282528081340622, 0.000800223746856249, 0},
//                                          {-0.317613,          0.088368,          -0.001007,            -0.000789,             0}};

// /*相机内外参   此处为8号门51，50ip的相机参数*/
// const float camera_param_in[2][9]     = {{1152.4, 0, 959.409, 0, 1147.4, 550.065, 0, 0, 1},
//                                          {1147.9, 0, 951.689, 0, 1143.4, 566.33,  0, 0, 1}};
// const float camera_param_rotate[2][3] = {{1.85233,  0.624684, -0.484852},
//                                          {2.02309,  0.128325, -0.105201}};
// const float camera_param_trans[2][3]  = {{234.929,  151.025, -0.075},
//                                          {95.262,   164.969,  15.345}};
// const float camera_param_dis[2][5]    = {{-0.3341,  0.1085,  0.000413,   0.00070638, 0},
//  {-0.3335,  0.109,  -0.00068918, 0.0013,     0}};

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

// // 视频回调函数（测试使用）
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

// // 视频回调函数（测试使用）
void test_cb(const CAlgResult& ms, void* hd)
{
    std::cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&& test_vb &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& \n";
}

void test()
{
    std::string img_path = "/share/FusionAliglib/data/video_data/";
    //算法接口调用流程基本如下：
    ITestAlg* l_pObj = CreateTestAlgObj("/share/FusionAliglib/Output");
    //准备算法参数
    CSelfAlgParam *l_stTestAlgParam = new CSelfAlgParam();
    l_stTestAlgParam->m_strRootPath = "/share/FusionAliglib/Output";
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
// //测试算法接口调用流程
int main(int argc, char* argv[])
{
    // testVideo();
    test();
    return 0;
}