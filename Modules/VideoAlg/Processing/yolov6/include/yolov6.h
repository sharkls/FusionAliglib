#include <fstream>
#include <iostream>
#include <sstream>
#include <numeric>
#include <chrono>
#include <vector>
#include <opencv2/opencv.hpp>
#include <dirent.h>
#include "NvInfer.h"
#include "cuda_runtime_api.h"
#include "logging_v5.h"
#include "yaml-cpp/yaml.h"

#include <opencv2/core/mat.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "CSelfAlgParam.h"
#include "xtensor/xarray.hpp"
#include "ICommonAlg.h"
using namespace nvinfer1;

using namespace std;


#define DEVICE 0  // GPU id
#define NMS_THRESH 0.45
#define BBOX_CONF_THRESH 0.5
// stuff we know about the network and the input/output blobs
const int num_class = 18;
static const int INPUT_W = 640;
static const int INPUT_H = 640;



class yolov6 : public ICommonAlg {
public:
    yolov6();
    yolov6(CSelfAlgParam *m_AlgParams);
    ~yolov6();

    void init(CSelfAlgParam *m_AlgParams);

    void execute();
 
private:
    // std::vector<std::string> class_names = {
    //     "person", "tool vehicle", "bicycle", "motorbike", "tricycle", "car", "Passenger car", "truck", "police car",
    //     "ambulance", "bus", "Dump truck", "tanker", "roadblock", "Fire car", "construction sign-board", "spillage", "rider"
    // };

    float* blob = nullptr;

    IRuntime* runtime;
    ICudaEngine* engine;
    IExecutionContext* context;
    int output_size = 1;
    float* prob;
    float scale;
    int img_w ;
    int img_h ;
    cv::Mat img; 
    void img_preprocess(cv::Mat& img); 
    std::string yolov6_weight;

    int Camera_num;
};


