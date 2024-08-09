#include "yolov5.h"
#include "config_v5.h"
#include <cstring>
#include "xtensor/xbuilder.hpp"
#include "xtensor/xtensor_forward.hpp"
#include "xtensor/xview.hpp"
#include "Log/glog/logging.h"

using namespace nvinfer1;

/*
*    功能：为输入和输出张量在GPU和CPU上分批内存
*    参数:
*       engine : yolov5模型的CUDA引擎
*       gpu_input_buffer : GPU上存储输入张量的数据
*       gpu_output_buffer : GPU上存储输出张量的数据
*       cpu_output_buffer : CPU上存储输出张量的数据
*/
void prepare_buffers(ICudaEngine *engine, float **gpu_input_buffer,
                     float **gpu_output_buffer, float **cpu_output_buffer) {
  assert(engine->getNbBindings() == 2);
  // In order to bind the buffers, we need to know the names of the input and
  // output tensors. Note that indices are guaranteed to be less than
  // IEngine::getNbBindings()
  const int inputIndex = engine->getBindingIndex(kInputTensorName);
  const int outputIndex = engine->getBindingIndex(kOutputTensorName);
  assert(inputIndex == 0);
  assert(outputIndex == 1);
  // Create GPU buffers on device
  CUDA_CHECK(cudaMalloc((void **)gpu_input_buffer,
                        kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
  CUDA_CHECK(cudaMalloc((void **)gpu_output_buffer,
                        kBatchSize * kOutputSize * sizeof(float)));

  *cpu_output_buffer = new float[kBatchSize * kOutputSize];
}

/*
*    功能：执行yolov5模型的推理，并将结果拷贝到主机内存
*    参数:
*       context : 用于执行推理特定的上下文
*       stream : 执行yolov5的CUDA流
*       gpu_buffers : GPU上存储张量的数据，输入和输出
*       output : 主机内存上的缓冲区，用于存放模型输出张量的数据
*       batchsize : 批处理大小
*/
void infer(IExecutionContext &context, cudaStream_t &stream, void **gpu_buffers,
           float *output, int batchsize) {
  context.enqueue(batchsize, gpu_buffers, stream, nullptr);
  CUDA_CHECK(cudaMemcpyAsync(output, gpu_buffers[1],
                             batchsize * kOutputSize * sizeof(float),
                             cudaMemcpyDeviceToHost, stream));
  cudaStreamSynchronize(stream);
}

/*
*    功能：创建并序列化ICudaEngine对象，并将序列化后的数据保存到文件中
*    参数:
*       max_batchsize : 最大的批处理大小
*       is_p6 : 是否使用P6引擎
*       gd : 模型深度因子
*       gw : 模型宽度乘因子
*       wts_name : 权重文件的名称
*       engine_name : 引擎文件的名称
*/
void serialize_engine(unsigned int max_batchsize, bool &is_p6, float &gd,
                      float &gw, std::string &wts_name,
                      std::string &engine_name) {
  // Create builder
  IBuilder *builder = createInferBuilder(gLogger);
  IBuilderConfig *config = builder->createBuilderConfig();

  // Create model to populate the network, then set the outputs and create an
  // engine
  ICudaEngine *engine = nullptr;
  if (is_p6) {
    engine = build_det_p6_engine(max_batchsize, builder, config,
                                 DataType::kFLOAT, gd, gw, wts_name);
  } else {
    engine = build_det_engine(max_batchsize, builder, config, DataType::kFLOAT,
                              gd, gw, wts_name);
  }
  assert(engine != nullptr);

  // Serialize the engine
  IHostMemory *serialized_engine = engine->serialize();
  assert(serialized_engine != nullptr);

  // Save engine to file
  std::ofstream p(engine_name, std::ios::binary);
  if (!p) {
    std::cerr << "Could not open plan output file" << std::endl;
    assert(false);
  }
  p.write(reinterpret_cast<const char *>(serialized_engine->data()),
          serialized_engine->size());

  // Close everything down
  engine->destroy();
  builder->destroy();
  config->destroy();
  serialized_engine->destroy();
}

/*
*    功能：从文件中读取序列化的ICudaEngine数据，并反序列化数据创建一个ICudaEngine对象和一个IExecutionContext对象。
*    参数:
*       engine_name : 引擎文件的名称
*       runtime : 指向IRuntime对象的指针的指针，IRuntime对象用于创建和销毁ICudaEngine对象
*       engine : 指向ICudaEngine对象的指针的指针，ICudaEngine对象包含了执行推理所需的所有信息
*       context : 指向IExecutionContext对象的指针的指针，IExecutionContext对象表示一个特定的上下文，用于执行推理
*/
void deserialize_engine(std::string &engine_name, IRuntime **runtime,
                        ICudaEngine **engine, IExecutionContext **context) {
  std::ifstream file(engine_name, std::ios::binary);
  if (!file.good()) {
    std::cerr << "read " << engine_name << " error!" << std::endl;
    assert(false);
  }
  size_t size = 0;
  file.seekg(0, file.end);
  size = file.tellg();
  file.seekg(0, file.beg);
  char *serialized_engine = new char[size];
  assert(serialized_engine);
  file.read(serialized_engine, size);
  file.close();

  *runtime = createInferRuntime(gLogger);
  assert(*runtime);
  *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
  assert(*engine);
  *context = (*engine)->createExecutionContext();
  assert(*context);
  delete[] serialized_engine;
}


/*
*    功能：yolov5的构造函数，构造yolov5检测器，并调用InitAlgorithm初始化模型
*   参数:
*       m_AlgParams : 视频算法参数
*/
yolov5::yolov5() 
{ 
  LOG(INFO) << "InitAlgorithm--yolov5 -> start " << std::endl; 
}

/*
*    功能：yolov5的析构函数，销毁CUDA流，释放GPU和CPU上分配的内存
*/
yolov5::~yolov5() {
  cudaStreamDestroy(stream);
  CUDA_CHECK(cudaFree(gpu_buffers[0]));
  CUDA_CHECK(cudaFree(gpu_buffers[1]));
  delete[] cpu_output_buffer;
  cuda_preprocess_destroy();
  // Destroy the engine
  context->destroy();
  engine->destroy();
  runtime->destroy();
}



void yolov5::init(CSelfAlgParam *p_pAlgParam) 
{ 
  std::cout << "yolov5 init start!" << std::endl;

  // 配置文件读取
  YAML::Node video_cfg = YAML::LoadFile(p_pAlgParam->m_strRootPath + p_pAlgParam->m_strVideoCfgPath);
  
  // 参数设置
  InputW = video_cfg["COMPRESS_IMG_W"].as<int>();  
  InputH = video_cfg["COMPRESS_IMG_H"].as<int>();   
  yolov5_type = video_cfg["YOLOV5_NET_TYPE"].as<std::string>();
  yolov5_weight = p_pAlgParam->m_strRootPath + video_cfg["YOLOV5_WEIGHT"].as<std::string>();
  
  if (yolov5_type == "n")
  {
    gd = 0.33;
    gw = 0.25;
  }
  else if (yolov5_type == "s")
  {
    gd = 0.33;
    gw = 0.50;
  }
  else if (yolov5_type == "m")
  {
    gd = 0.67;
    gw = 0.75;
  }
  else if (yolov5_type == "l")
  {
    gd = 1.0;
    gw = 1.0;
  }
  else if (yolov5_type == "x")
  {
    gd = 1.33;
    gw = 1.25;
  }
  else
  {
    return;
  }

  if (yolov5_weight.empty())
  {
    std::cout << "Load Failed. Please check the weight or engine path.\n";
  }
  int split_index = yolov5_weight.find_last_of('.');

  std::string file_type = yolov5_weight.substr(split_index+ 1, yolov5_weight.size() - split_index - 1);
  if (file_type == "wts")
  {
    wts_name = yolov5_weight;
    engine_name = yolov5_weight.substr(0, split_index + 1) + "engine";
  }
  else
  {
    engine_name = yolov5_weight;
  }
  if (access(engine_name.data(), F_OK) != 0)
  {
    if (access(wts_name.data(), F_OK) != 0)
    {
      std::cout<< "The .wts and .engine both not found.\n";
      return;
    }
    else
    {
      std::cout<< "serialize_engine start !.\n";
      serialize_engine(kBatchSize, is_p6, gd, gw, wts_name, engine_name);
      std::cout<< "serialize_engine done !.\n";
    }
  }

  deserialize_engine(engine_name, &runtime, &engine, &context);
  
  CUDA_CHECK(cudaStreamCreate(&stream));
  cuda_preprocess_init(kMaxInputImageSize);
  
  prepare_buffers(engine, &gpu_buffers[0], &gpu_buffers[1], &cpu_output_buffer);

  LOG(INFO) << "InitAlgorithm--yolov5 -> finish! " << std::endl; 
}




void yolov5::execute()
{
  LOG(INFO) << "RunAlgorithm--yolov5 -> start " << std::endl; 
  auto t_start = std::chrono::high_resolution_clock::now();

  // 1. 图像数据获取
  std::vector<cv::Mat> img_batch = static_cast<std::vector<cv::Mat>>(getCommonData()->img_batch);
  if (img_batch.size() <= 0){                                                   // 图像推理模块输入数据为零时，将输出设置为空指针并跳过推理部分
    LOG(ERROR) << "yolov5::execute ----  img_batch is None  " << std::endl;
    m_CommonData->cpu_buffer = nullptr;
    return;   
  }

  // 2.仿射变换
  cuda_batch_preprocess(img_batch, gpu_buffers[0], InputW, InputH, stream);

  // Run inference
  infer(*context, stream, (void **)gpu_buffers, cpu_output_buffer, img_batch.size());       // 可变batch_size

  // 设置检测输出结果
  m_CommonData->cpu_buffer = cpu_output_buffer;
  auto t_end = std::chrono::high_resolution_clock::now();
  auto latency_sp = (std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_end - t_start).count())/1000;
  LOG(INFO) << "RunAlgorithm--yolov5 -> finish! >>> All Time :"<< latency_sp << " ms"  << std::endl;
}