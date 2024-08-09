#include "yolov6.h"
#include "xtensor/xbuilder.hpp"
#include "xtensor/xtensor_forward.hpp"
#include "xtensor/xview.hpp"
#include "Log/glog/logging.h"

const char* INPUT_BLOB_NAME = "image_arrays";
const char* OUTPUT_BLOB_NAME = "outputs";

//#include "NvInferPlugin.h"
//auto initP = initLibNvInferPlugins(nullptr,"");
#define CHECK(status) \
    do\
    {\
        auto ret = (status);\
        if (ret != 0)\
        {\
            std::cerr << "Cuda failure: " << ret << std::endl;\
            abort();\
        }\
    } while (0)

static Logger gLogger;


cv::Mat static_resize(cv::Mat& img) {
    float r = std::min(INPUT_W / (img.cols*1.0), INPUT_H / (img.rows*1.0));
    int unpad_w = r * img.cols;
    int unpad_h = r * img.rows;
    cv::Mat re(unpad_h, unpad_w, CV_8UC3);
    cv::resize(img, re, re.size());
    cv::Mat out(INPUT_W, INPUT_H, CV_8UC3, cv::Scalar(114, 114, 114));
    re.copyTo(out(cv::Rect(0, 0, re.cols, re.rows)));
    return out;
}

struct Object
{
    cv::Rect_<float> rect;
    int label;
    float prob;
};


static inline float intersection_area(const Object& a, const Object& b)
{
    cv::Rect_<float> inter = a.rect & b.rect;
    return inter.area();
}

static void qsort_descent_inplace(std::vector<Object>& faceobjects, int left, int right)
{
    int i = left;
    int j = right;
    float p = faceobjects[(left + right) / 2].prob;

    while (i <= j)
    {
        while (faceobjects[i].prob > p)
            i++;

        while (faceobjects[j].prob < p)
            j--;

        if (i <= j)
        {
            // swap
            std::swap(faceobjects[i], faceobjects[j]);

            i++;
            j--;
        }
    }

    #pragma omp parallel sections
    {
        #pragma omp section
        {
            if (left < j) qsort_descent_inplace(faceobjects, left, j);
        }
        #pragma omp section
        {
            if (i < right) qsort_descent_inplace(faceobjects, i, right);
        }
    }
}

static void qsort_descent_inplace(std::vector<Object>& objects)
{
    if (objects.empty())
        return;

    qsort_descent_inplace(objects, 0, objects.size() - 1);
}

/*
*    功能：对预测的边界框进行非极大值抑制（NMS）操作，以消除重叠的预测框
*    参数:
*        faceobjects : 预测边界框集合
*        picked : 模型输出的大小
*        nms_threshold : 置信度阈值
*/
static void nms_sorted_bboxes(const std::vector<Object>& faceobjects, std::vector<int>& picked, float nms_threshold)
{
    picked.clear();

    const int n = faceobjects.size();

    std::vector<float> areas(n);
    for (int i = 0; i < n; i++)
    {
        areas[i] = faceobjects[i].rect.area();
    }

    for (int i = 0; i < n; i++)
    {
        const Object& a = faceobjects[i];

        int keep = 1;
        for (int j = 0; j < (int)picked.size(); j++)
        {
            const Object& b = faceobjects[picked[j]];

            // intersection over union
            float inter_area = intersection_area(a, b);
            float union_area = areas[i] + areas[picked[j]] - inter_area;
            // float IoU = inter_area / union_area
            if (inter_area / union_area > nms_threshold)
                keep = 0;
        }

        if (keep)
            picked.push_back(i);
    }
}

/*
*    功能：解析模型的输出，生成预测的边界框，并基于置信度阈值进行筛选。
*    参数:
*        feat_blob : 模型输出结果
*        output_size : 模型输出的大小
*        prob_threshold : 置信度阈值
*        objects : 模型输出的预测边界框
*/
static void generate_yolo_proposals(float* feat_blob, int output_size, float prob_threshold, std::vector<Object>& objects)
{
    auto dets = output_size / (num_class + 5);
    for (int boxs_idx = 0; boxs_idx < dets; boxs_idx++)
    {
        const int basic_pos = boxs_idx *(num_class + 5);
        float x_center = feat_blob[basic_pos+0];
        float y_center = feat_blob[basic_pos+1];
        float w = feat_blob[basic_pos+2];
        float h = feat_blob[basic_pos+3];
        float x0 = x_center - w * 0.5f;
        float y0 = y_center - h * 0.5f;
        float box_objectness = feat_blob[basic_pos+4];
        // std::cout<<*feat_blob<<std::endl;
        for (int class_idx = 0; class_idx < num_class; class_idx++)
        {
            float box_cls_score = feat_blob[basic_pos + 5 + class_idx];
            float box_prob = box_objectness * box_cls_score;
            if (box_prob > prob_threshold)
            {
                Object obj;
                obj.rect.x = x0;
                obj.rect.y = y0;
                obj.rect.width = w;
                obj.rect.height = h;
                obj.label = class_idx;
                obj.prob = box_prob;

                objects.push_back(obj);
            }

        } // class loop
    }

}

/*
*    功能：将输入的图像转换为blob格式，图像格式转化：BRG->RGB , 像素值归一化
*    参数:
*        img : 输入的图像数据
*    返回：
*        blob : blob格式的数据
*/
float* blobFromImage(cv::Mat& img){
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

    float* blob = new float[img.total()*3];
    int channels = 3;
    int img_h = img.rows;
    int img_w = img.cols;
    for (size_t c = 0; c < channels; c++)
    {
        for (size_t  h = 0; h < img_h; h++)
        {
            for (size_t w = 0; w < img_w; w++)
            {
                blob[c * img_w * img_h + h * img_w + w] =
                    (((float)img.at<cv::Vec3b>(h, w)[c]) / 255.0f);
            }
        }
    }
    return blob;
}

/*
*    功能：对模型感知输出进行解析，获取预测的边界框并基于置信度大小进行排序，NMS过滤，放缩为原始图像大小
*    参数:
*        prob : yolov6模型输出结果
*        output_size : 模型输出类别数量
*        objects : 处理后的目标信息
*        scale : 尺度
*        img_w : 图像的原始宽度
*        img_h : 图像的原始高度
*/
static void decode_outputs(float* prob, int output_size, std::vector<Object>& objects, float scale, const int img_w, const int img_h) {
    // auto postprocess_start = std::chrono::system_clock::now();

    std::vector<Object> proposals;
    generate_yolo_proposals(prob, output_size, BBOX_CONF_THRESH, proposals);

    qsort_descent_inplace(proposals);

    std::vector<int> picked;
    nms_sorted_bboxes(proposals, picked, NMS_THRESH);

    int count = picked.size();

    objects.resize(count);
    for (int i = 0; i < count; i++)
    {
        objects[i] = proposals[picked[i]];

        // adjust offset to original unpadded
        float x0 = (objects[i].rect.x) / scale;
        float y0 = (objects[i].rect.y) / scale;
        float x1 = (objects[i].rect.x + objects[i].rect.width) / scale;
        float y1 = (objects[i].rect.y + objects[i].rect.height) / scale;

        // clip
        x0 = std::max(std::min(x0, (float)(img_w - 1)), 0.f);
        y0 = std::max(std::min(y0, (float)(img_h - 1)), 0.f);
        x1 = std::max(std::min(x1, (float)(img_w - 1)), 0.f);
        y1 = std::max(std::min(y1, (float)(img_h - 1)), 0.f);

        objects[i].rect.x = x0;
        objects[i].rect.y = y0;
        objects[i].rect.width = x1 - x0;
        objects[i].rect.height = y1 - y0;
    }

    
    // auto postprocess_end = std::chrono::system_clock::now();
    // double postprocess_time = std::chrono::duration_cast<std::chrono::milliseconds>(postprocess_end - postprocess_start).count();
    // std::cout <<"Yolov6 ---- postprocess Time:"<< postprocess_time << " ms" << std::endl;
}

//   doInference(*context, blob, prob, output_size, pr_img.size());
/*
*    功能：执行模型的推理，并将结果从GPU内存复制到主机内存
*    参数:
*        context : 特定的上下文，用于执行推理
*        input : 指向主机内存中的输入数据的指针
*        output : 指向主机内存中的输出数据的指针
*        output_size : 输出数据的大小
*        input_shape : 输入数据的形状
*/
void doInference(IExecutionContext& context, float* input, float* output, const int output_size, cv::Size input_shape) {
    // auto inference_start = std::chrono::system_clock::now();
    const ICudaEngine& engine = context.getEngine();
    assert(engine.getNbBindings() == 2);
    void* buffers[2];

    const int inputIndex = engine.getBindingIndex(INPUT_BLOB_NAME);

    assert(engine.getBindingDataType(inputIndex) == nvinfer1::DataType::kFLOAT);    // nvinfer1::DataType::kFLOAT = 数据类型

    const int outputIndex = engine.getBindingIndex(OUTPUT_BLOB_NAME);
    assert(engine.getBindingDataType(outputIndex) == nvinfer1::DataType::kFLOAT);

    int mBatchSize = engine.getMaxBatchSize();

    CHECK(cudaMalloc(&buffers[inputIndex], 3 * input_shape.height * input_shape.width * sizeof(float)));
    CHECK(cudaMalloc(&buffers[outputIndex], output_size*sizeof(float)));
    // Create stream
    cudaStream_t stream;
    CHECK(cudaStreamCreate(&stream));
    CHECK(cudaMemcpyAsync(buffers[inputIndex], input, 3 * input_shape.height * input_shape.width * sizeof(float), cudaMemcpyHostToDevice, stream));
    
    context.enqueue(1, buffers, stream, nullptr);
    CHECK(cudaMemcpyAsync(output, buffers[outputIndex], output_size * sizeof(float), cudaMemcpyDeviceToHost, stream));

    cudaStreamSynchronize(stream);
    cudaStreamDestroy(stream);
    CHECK(cudaFree(buffers[inputIndex]));
    CHECK(cudaFree(buffers[outputIndex]));
    // auto inference_end = std::chrono::system_clock::now();
    // double inference_time = std::chrono::duration_cast<std::chrono::milliseconds>(inference_end - inference_start).count();
    // std::cout <<"Yolov6 ---- inference Time:"<< inference_time << " ms" << std::endl;
}


/*
*    功能：yolov6的构造函数，构造yolov6检测器，并调用InitAlgorithm初始化模型
*   参数:
*       m_AlgParams : 视频算法参数
*/
yolov6::yolov6()
{
    LOG(INFO) << "InitAlgorithm--yolov6 -> start " << std::endl; 
}


/*
*    功能：yolov6的析构函数，销毁CUDA流，释放GPU和CPU上分配的内存
*/
yolov6::~yolov6(){
    context->destroy();
    engine->destroy();
    runtime->destroy();
    delete blob;
    delete prob;
}


/*
*    功能：yolov6的初始化函数，加载yolov6模型结构参数和权重文件，搭建yolov6模型并序列化
*    参数:
*         m_AlgParams : 视频算法参数
*/
void yolov6::init(CSelfAlgParam *m_AlgParams) 
{
    YAML::Node video_cfg = YAML::LoadFile(m_AlgParams->m_strRootPath + m_AlgParams->m_strVideoCfgPath);
    m_AlgParams->m_vecVideoClass = {video_cfg["YOLOV6_CLASS_NAMES"][0].as<std::string>(),
            video_cfg["YOLOV6_CLASS_NAMES"][1].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][2].as<std::string>(),
            video_cfg["YOLOV6_CLASS_NAMES"][3].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][4].as<std::string>(),
            video_cfg["YOLOV6_CLASS_NAMES"][5].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][6].as<std::string>(),
            video_cfg["YOLOV6_CLASS_NAMES"][7].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][8].as<std::string>(),
            video_cfg["YOLOV6_CLASS_NAMES"][9].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][10].as<std::string>(),
            video_cfg["YOLOV6_CLASS_NAMES"][11].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][12].as<std::string>(),
            video_cfg["YOLOV6_CLASS_NAMES"][13].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][14].as<std::string>(),
            video_cfg["YOLOV6_CLASS_NAMES"][15].as<std::string>(), video_cfg["YOLOV6_CLASS_NAMES"][16].as<std::string>(),
            video_cfg["YOLOV6_CLASS_NAMES"][17].as<std::string>()};
    yolov6_weight = m_AlgParams->m_strRootPath + video_cfg["YOLOV6_WEIGHT"].as<std::string>();

    Camera_num = m_AlgParams->m_nCameraNum;     // 获取相机数量

    cudaSetDevice(DEVICE);
    char *trtModelStream{nullptr};
    size_t size{0};
    std::ifstream file(yolov6_weight, std::ios::binary);
    if (file.good()) {
        file.seekg(0, file.end);    //让文件指针定位到文件末尾
        size = file.tellg();        //得到文件指针当前指向的文件位置
        file.seekg(0, file.beg);    //让文件指针定位到文件开头
        trtModelStream = new char[size];
        assert(trtModelStream);
        file.read(trtModelStream, size);
        file.close();
    }
    else{
        std::cout <<"Yolov6 engine model path is empty."<< std::endl;
        return;
    }

    runtime = createInferRuntime(gLogger);
    assert(runtime != nullptr);
    engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    context = engine->createExecutionContext();
    assert(context != nullptr);
    delete[] trtModelStream;

    auto out_dims = engine->getBindingDimensions(1);

    for(int j=0;j<out_dims.nbDims;j++) {
        output_size *= out_dims.d[j];
    }
    prob = new float[output_size];
}


/*
*    功能：yolov6的预处理函数，将图像进行Resize
*    参数:
*        img : 单路或者多路相机图像
*    返回: 
*        out_put : yolov6 检测结果
*/
void yolov6::img_preprocess(cv::Mat& img){
    // auto preprocess_start = std::chrono::system_clock::now();
    img_w = img.cols;
    img_h = img.rows;
    scale = std::min(INPUT_W / (img.cols*1.0), INPUT_H / (img.rows*1.0));
    img = static_resize(img);    // resize to 640*640
    blob = blobFromImage(img);   
    // auto preprocess_end = std::chrono::system_clock::now();
    // double preprocess_time = std::chrono::duration_cast<std::chrono::milliseconds>(preprocess_end - preprocess_start).count();
    // std::cout <<"Yolov6 ---- Preprocess Time:"<< preprocess_time << " ms" << std::endl;
    
}



/*
*    功能：yolov6的推理函数，接收单路或多路图像数据，基于构建的模型进行前向推理，并返回检测结果
*    参数:
*        img_batch : 单路或者多路相机图像
*    返回: 
*        out_put : yolov6 检测结果
*/
// std::vector<xt::xarray<float>> yolov6::RunAlgorithm(std::vector<cv::Mat> & img_batch){
void yolov6::execute(){
    std::vector<cv::Mat> * img_batch = static_cast<std::vector<cv::Mat> *>(getBufferElement());

    std::vector<xt::xarray<float>> out_put(Camera_num);

    
    for (int i = 0; i < img_batch->size(); ++i)
    {
        img = (*img_batch)[i];
        img_preprocess(img);
        doInference(*context, blob, prob, output_size, img.size());
        std::vector<Object> objects;
        decode_outputs(prob, output_size, objects, scale, img_w, img_h);
    }


    setBufferElement(& prob);

}