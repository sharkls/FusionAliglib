#include "calibrator_v5.h"
#include "cuda_utils_v5.h"
#include "utils_v5.h"

#include <iostream>
#include <iterator>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>

/*
*    功能：预处理图像，将图像缩放到指定大小，并在必要时进行填充以保持长宽比
*    参数:
*       img : 原始图像
*       input_w : 目标图像宽度
*       input_h : 目标图像高度
*    返回:
*       out : 预处理后的图像
*/
static cv::Mat preprocess_img(cv::Mat& img, int input_w, int input_h) {
  int w, h, x, y;
  float r_w = input_w / (img.cols * 1.0);
  float r_h = input_h / (img.rows * 1.0);
  if (r_h > r_w) {
    w = input_w;
    h = r_w * img.rows;
    x = 0;
    y = (input_h - h) / 2;
  } else {
    w = r_h * img.cols;
    h = input_h;
    x = (input_w - w) / 2;
    y = 0;
  }
  cv::Mat re(h, w, CV_8UC3);
  cv::resize(img, re, re.size(), 0, 0, cv::INTER_LINEAR);
  cv::Mat out(input_h, input_w, CV_8UC3, cv::Scalar(128, 128, 128));
  re.copyTo(out(cv::Rect(x, y, re.cols, re.rows)));
  return out;
}

/*
*    功能：Int8EntropyCalibrator2的构造函数，初始化校准器参数
*    参数:
*       batchsize : 批处理大小
*       input_w : 输入图像的宽度
*       input_h : 输入图像的高度
*       img_dir : 包含用于校准的图像的目录路径
*       calib_table_name : 校准表的文件名
*       input_blob_name : 网络输入blob的名称
*       read_cache : 是否读取缓存的校准表
*/
Int8EntropyCalibrator2::Int8EntropyCalibrator2(int batchsize, int input_w, int input_h, const char* img_dir, const char* calib_table_name, const char* input_blob_name, bool read_cache)
    : batchsize_(batchsize),
      input_w_(input_w),
      input_h_(input_h),
      img_idx_(0),
      img_dir_(img_dir),
      calib_table_name_(calib_table_name),
      input_blob_name_(input_blob_name),
      read_cache_(read_cache) {
  input_count_ = 3 * input_w * input_h * batchsize;
  CUDA_CHECK(cudaMalloc(&device_input_, input_count_ * sizeof(float)));
  read_files_in_dir(img_dir, img_files_);
}

/*
*    功能：Int8EntropyCalibrator2的析构函数，释放CUDA设备上分配的内存
*/
Int8EntropyCalibrator2::~Int8EntropyCalibrator2() {
  CUDA_CHECK(cudaFree(device_input_));
}

/*
*    功能：获取批处理大小
*    返回:
*       batchsize_ : 当前校准器的批处理大小
*/
int Int8EntropyCalibrator2::getBatchSize() const TRT_NOEXCEPT {
  return batchsize_;
}

/*
*    功能：获取一批图像数据并将其传输到设备内存中
*    参数:
*       bindings : 指向网络输入和输出内存的指针数组
*       names : 网络输入和输出blob的名称数组
*       nbBindings : 网络输入和输出blob的数量
*    返回:
*       成功获取批处理数据返回true，否则返回false
*/
bool Int8EntropyCalibrator2::getBatch(void* bindings[], const char* names[], int nbBindings) TRT_NOEXCEPT {
  if (img_idx_ + batchsize_ > (int)img_files_.size()) {
    return false;
  }

  std::vector<cv::Mat> input_imgs_;
  for (int i = img_idx_; i < img_idx_ + batchsize_; i++) {
    std::cout << img_files_[i] << "  " << i << std::endl;
    cv::Mat temp = cv::imread(img_dir_ + img_files_[i]);
    if (temp.empty()) {
      std::cerr << "Fatal error: image cannot open!" << std::endl;
      return false;
    }
    cv::Mat pr_img = preprocess_img(temp, input_w_, input_h_);
    input_imgs_.push_back(pr_img);
  }
  img_idx_ += batchsize_;
  cv::Mat blob = cv::dnn::blobFromImages(input_imgs_, 1.0 / 255.0, cv::Size(input_w_, input_h_), cv::Scalar(0, 0, 0), true, false);

  CUDA_CHECK(cudaMemcpy(device_input_, blob.ptr<float>(0), input_count_ * sizeof(float), cudaMemcpyHostToDevice));
  assert(!strcmp(names[0], input_blob_name_));
  bindings[0] = device_input_;
  return true;
}

/*
*    功能：读取校准缓存
*    参数:
*       length : 校准缓存的长度
*    返回:
*       校准缓存的数据指针，如果没有缓存则返回nullptr
*/
const void* Int8EntropyCalibrator2::readCalibrationCache(size_t& length) TRT_NOEXCEPT {
  std::cout << "reading calib cache: " << calib_table_name_ << std::endl;
  calib_cache_.clear();
  std::ifstream input(calib_table_name_, std::ios::binary);
  input >> std::noskipws;
  if (read_cache_ && input.good()) {
    std::copy(std::istream_iterator<char>(input), std::istream_iterator<char>(), std::back_inserter(calib_cache_));
  }
  length = calib_cache_.size();
  return length ? calib_cache_.data() : nullptr;
}

/*
*    功能：写入校准缓存
*    参数:
*       cache : 指向校准缓存数据的指针
*       length : 校准缓存数据的长度
*/
void Int8EntropyCalibrator2::writeCalibrationCache(const void* cache, size_t length) TRT_NOEXCEPT {
  std::cout << "writing calib cache: " << calib_table_name_ << " size: " << length << std::endl;
  std::ofstream output(calib_table_name_, std::ios::binary);
  output.write(reinterpret_cast<const char*>(cache), length);
}


