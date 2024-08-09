#pragma once

#include "cuda_utils_v5.h"
#include "logging_v5.h"
#include "model_v5.h"
#include "postprocess_v5.h"
#include "preprocess_v5.h"
#include "utils_v5.h"
#include "CSelfAlgParam.h"
// #include "xtensor/xarray.hpp"
#include "yaml-cpp/yaml.h"
#include "ICommonAlg.h"

#include <unistd.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <string>
#include <vector>

using namespace nvinfer1;

static Logger gLogger;
const static int kOutputSize =
    kMaxNumOutputBbox * sizeof(yolov5Detection) / sizeof(float) + 1;

static int images_times = 1;

void prepare_buffers(ICudaEngine *engine, float **gpu_input_buffer,
                     float **gpu_output_buffer, float **cpu_output_buffer);

void infer(IExecutionContext &context, cudaStream_t &stream, void **gpu_buffers,
           float *output, int batchsize);

void serialize_engine(unsigned int max_batchsize, bool &is_p6, float &gd,
                      float &gw, std::string &wts_name,
                      std::string &engine_name);
void deserialize_engine(std::string &engine_name, IRuntime **runtime,
                        ICudaEngine **engine, IExecutionContext **context);



class yolov5 : public ICommonAlg {
public:
  yolov5();
  yolov5(CSelfAlgParam *m_AlgParams);
  ~yolov5();

  void init(CSelfAlgParam *p_pAlgParam);

  void execute();

  void setCommonData(CCommonDataPtr p_commonData)
    {
        m_CommonData = p_commonData;
    }

  CCommonDataPtr getCommonData() 
    {
        return m_CommonData;
    }

private:
  void standard_box(std::vector<std::vector<yolov5Detection>> &input);
  bool is_p6 = false;
  float gd;
  float gw;
  int InputW;
  int InputH;
  IRuntime *runtime = nullptr;
  ICudaEngine *engine = nullptr;
  IExecutionContext *context = nullptr;

  cudaStream_t stream;
  float *gpu_buffers[2];
  float *cpu_output_buffer = nullptr;
  std::string wts_name;
  std::string engine_name;
  std::string yolov5_type;
  std::string yolov5_weight;
};

