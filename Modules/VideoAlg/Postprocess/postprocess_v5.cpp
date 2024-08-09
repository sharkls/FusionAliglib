#include "postprocess_v5.h"
#include "utils_v5.h"

/*
* 功能：根据图像和边界框数组计算矩形框
* 参数:
*   img: 参考图像
*   bbox: 边界框数组，包含中心点坐标和宽高
* 返回:
*   计算得到的矩形框
*/
cv::Rect get_rect(cv::Mat& img, float bbox[4]) {
  float l, r, t, b;
  float r_w = kInputW / (img.cols * 1.0);
  float r_h = kInputH / (img.rows * 1.0);
  if (r_h > r_w) {
    l = bbox[0] - bbox[2] / 2.f;
    r = bbox[0] + bbox[2] / 2.f;
    t = bbox[1] - bbox[3] / 2.f - (kInputH - r_w * img.rows) / 2;
    b = bbox[1] + bbox[3] / 2.f - (kInputH - r_w * img.rows) / 2;
    l = l / r_w;
    r = r / r_w;
    t = t / r_w;
    b = b / r_w;
  } else {
    l = bbox[0] - bbox[2] / 2.f - (kInputW - r_h * img.cols) / 2;
    r = bbox[0] + bbox[2] / 2.f - (kInputW - r_h * img.cols) / 2;
    t = bbox[1] - bbox[3] / 2.f;
    b = bbox[1] + bbox[3] / 2.f;
    l = l / r_h;
    r = r / r_h;
    t = t / r_h;
    b = b / r_h;
  }
  return cv::Rect(round(l), round(t), round(r - l), round(b - t));
}

/*
* 功能：计算两个边界框的交并比（IoU）
* 参数:
*   lbox: 第一个边界框数组
*   rbox: 第二个边界框数组
* 返回:
*   交并比值
*/
static float iou(float lbox[4], float rbox[4]) {
  float interBox[] = {
    (std::max)(lbox[0] - lbox[2] / 2.f , rbox[0] - rbox[2] / 2.f), //left
    (std::min)(lbox[0] + lbox[2] / 2.f , rbox[0] + rbox[2] / 2.f), //right
    (std::max)(lbox[1] - lbox[3] / 2.f , rbox[1] - rbox[3] / 2.f), //top
    (std::min)(lbox[1] + lbox[3] / 2.f , rbox[1] + rbox[3] / 2.f), //bottom
  };

  if (interBox[2] > interBox[3] || interBox[0] > interBox[1])
    return 0.0f;

  float interBoxS = (interBox[1] - interBox[0])*(interBox[3] - interBox[2]);
  return interBoxS / (lbox[2] * lbox[3] + rbox[2] * rbox[3] - interBoxS);
}

/*
* 功能：比较两个检测结果的置信度
* 参数:
*   a: 第一个检测结果
*   b: 第二个检测结果
* 返回:
*   如果a的置信度大于b，则返回true
*/
static bool cmp(const yolov5Detection& a, const yolov5Detection& b) {
  return a.conf > b.conf;
}

/*
* 功能：执行非极大值抑制（NMS）
* 参数:
*   res: 检测结果向量的引用
*   output: 检测结果数组
*   conf_thresh: 置信度阈值
*   nms_thresh: NMS阈值
*/
// void nms(std::vector<yolov5Detection>& res, float* output, float conf_thresh, float nms_thresh) {

//   int det_size = sizeof(yolov5Detection) / sizeof(float);
//   std::map<float, std::vector<yolov5Detection>> m;

//   for (int i = 0; i < output[0] && i < kMaxNumOutputBbox; i++) {
//     if (output[1 + det_size * i + 4] <= conf_thresh) continue;
//     yolov5Detection det;
//     memcpy(&det, &output[1 + det_size * i], det_size * sizeof(float));
//     if (m.count(det.class_id) == 0) m.emplace(det.class_id, std::vector<yolov5Detection>());
//     m[det.class_id].push_back(det);
//   }
//   std::cout << "det Nums: " << m.size() << std::endl;
//   for (auto it = m.begin(); it != m.end(); it++) {
//     auto& dets = it->second;
//     std::sort(dets.begin(), dets.end(), cmp);
//     for (size_t m = 0; m < dets.size(); ++m) {
//       auto& item = dets[m];
//       res.push_back(item);
//       for (size_t n = m + 1; n < dets.size(); ++n) {
//         if (iou(item.bbox, dets[n].bbox) > nms_thresh) {
//           dets.erase(dets.begin() + n);
//           --n;
//         }
//       }
//     }
//   }
// }
void nms(std::vector<yolov5Detection>& res, float* output, std::vector<float> conf_thresholds, float nms_thresh) {

  int det_size = sizeof(yolov5Detection) / sizeof(float);
  std::map<float, std::vector<yolov5Detection>> m;

  for (int i = 0; i < output[0] && i < kMaxNumOutputBbox; i++) {
    yolov5Detection det;
    memcpy(&det, &output[1 + det_size * i], det_size * sizeof(float));
    // 确保类别ID在阈值数组范围内
    // std::cout << "00000000000000----: " << det.class_id << " conf_thresholds.size():" << conf_thresholds.size() << std::endl;
    if (det.class_id >= 0 && det.class_id < conf_thresholds.size()) {
      // 根据类别ID获取对应的置信度阈值
      float class_conf_thresh = conf_thresholds[det.class_id];
      if (det.conf <= class_conf_thresh) continue; // 根据类别的置信度阈值进行筛选
    } else {
      continue; // 类别ID不在阈值数组范围内，跳过此检测结果
    }
    if (m.count(det.class_id) == 0) m.emplace(det.class_id, std::vector<yolov5Detection>());
    m[det.class_id].push_back(det);
  }
  // std::cout << "det Nums: " << m.size() << std::endl;
  for (auto it = m.begin(); it != m.end(); it++) {
    auto& dets = it->second;
    std::sort(dets.begin(), dets.end(), cmp);
    for (size_t m = 0; m < dets.size(); ++m) {
      auto& item = dets[m];
      res.push_back(item);
      for (size_t n = m + 1; n < dets.size(); ++n) {
        if (iou(item.bbox, dets[n].bbox) > nms_thresh) {
          dets.erase(dets.begin() + n);
          --n;
        }
      }
    }
  }
}

/*
* 功能：批量执行非极大值抑制（NMS）
* 参数:
*   res_batch: 批量检测结果向量的引用
*   output: 批量检测结果数组
*   batch_size: 批量大小
*   output_size: 单个输出的大小
*   conf_thresh: 置信度阈值
*   nms_thresh: NMS阈值
*/
void batch_nms(std::vector<std::vector<yolov5Detection>>& res_batch, float *output, int batch_size, int output_size, std::vector<float> conf_thresh, float nms_thresh) {
  res_batch.resize(batch_size);
  for (int i = 0; i < batch_size; i++) {
    nms(res_batch[i], &output[i * output_size], conf_thresh, nms_thresh);
  }
}


/*
* 功能：在图像批量上绘制边界框
* 参数:
*   img_batch: 图像批量的向量
*   res_batch: 批量检测结果向量
*/
void draw_bbox(std::vector<cv::Mat>& img_batch, std::vector<std::vector<yolov5Detection>>& res_batch) {
  for (size_t i = 0; i < img_batch.size(); i++) {
    auto& res = res_batch[i];
    cv::Mat img = img_batch[i];
    for (size_t j = 0; j < res.size(); j++) {
      cv::Rect r = get_rect(img, res[j].bbox);
      cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
      cv::putText(img, std::to_string((int)res[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }
  }
}

/*
* 功能：根据缩放比例和边界框数组计算缩放后的矩形框
* 参数:
*   bbox: 边界框数组
*   scale: 缩放比例
* 返回:
*   缩放后的矩形框
*/
static cv::Rect get_downscale_rect(float bbox[4], float scale) {
  float left = bbox[0] - bbox[2] / 2;
  float top = bbox[1] - bbox[3] / 2;
  float right = bbox[0] + bbox[2] / 2;
  float bottom = bbox[1] + bbox[3] / 2;
  left /= scale;
  top /= scale;
  right /= scale;
  bottom /= scale;
  return cv::Rect(round(left), round(top), round(right - left), round(bottom - top));
}

/*
* 功能：处理掩码
* 参数:
*   proto: 原型数组
*   proto_size: 原型数组大小
*   dets: 检测结果向量
* 返回:
*   处理后的掩码矩阵向量
*/
std::vector<cv::Mat> process_mask(const float* proto, int proto_size, std::vector<yolov5Detection>& dets) {
  std::vector<cv::Mat> masks;
  for (size_t i = 0; i < dets.size(); i++) {
    cv::Mat mask_mat = cv::Mat::zeros(kInputH / 4, kInputW / 4, CV_32FC1);
    auto r = get_downscale_rect(dets[i].bbox, 4);
    for (int x = r.x; x < r.x + r.width; x++) {
      for (int y = r.y; y < r.y + r.height; y++) {
        float e = 0.0f;
        for (int j = 0; j < 32; j++) {
          e += dets[i].mask[j] * proto[j * proto_size / 32 + y * mask_mat.cols + x];
        }
        e = 1.0f / (1.0f + expf(-e));
        mask_mat.at<float>(y, x) = e;
      }
    }
    cv::resize(mask_mat, mask_mat, cv::Size(kInputW, kInputH));
    masks.push_back(mask_mat);
  }
  return masks;
}

/*
* 功能：缩放掩码以匹配图像尺寸
* 参数:
*   mask: 掩码矩阵
*   img: 参考图像
* 返回:
*   缩放后的掩码矩阵
*/
cv::Mat scale_mask(cv::Mat mask, cv::Mat img) {
  int x, y, w, h;
  float r_w = kInputW / (img.cols * 1.0);
  float r_h = kInputH / (img.rows * 1.0);
  if (r_h > r_w) {
    w = kInputW;
    h = r_w * img.rows;
    x = 0;
    y = (kInputH - h) / 2;
  } else {
    w = r_h * img.cols;
    h = kInputH;
    x = (kInputW - w) / 2;
    y = 0;
  }
  cv::Rect r(x, y, w, h);
  cv::Mat res;
  cv::resize(mask(r), res, img.size());
  return res;
}

/*
* 功能：在图像上绘制掩码和边界框，并标注类别和置信度
* 参数:
*   img: 输入图像
*   dets: 检测结果向量
*   masks: 掩码向量
*   labels_map: 类别标签映射
*/
void draw_mask_bbox(cv::Mat& img, std::vector<yolov5Detection>& dets, std::vector<cv::Mat>& masks, std::unordered_map<int, std::string>& labels_map) {
  static std::vector<uint32_t> colors = {0xFF3838, 0xFF9D97, 0xFF701F, 0xFFB21D, 0xCFD231, 0x48F90A,
                                         0x92CC17, 0x3DDB86, 0x1A9334, 0x00D4BB, 0x2C99A8, 0x00C2FF,
                                         0x344593, 0x6473FF, 0x0018EC, 0x8438FF, 0x520085, 0xCB38FF,
                                         0xFF95C8, 0xFF37C7};
  for (size_t i = 0; i < dets.size(); i++) {
    cv::Mat img_mask = scale_mask(masks[i], img);
    auto color = colors[(int)dets[i].class_id % colors.size()];
    auto bgr = cv::Scalar(color & 0xFF, color >> 8 & 0xFF, color >> 16 & 0xFF);

    cv::Rect r = get_rect(img, dets[i].bbox);
    for (int x = r.x; x < r.x + r.width; x++) {
      for (int y = r.y; y < r.y + r.height; y++) {
        float val = img_mask.at<float>(y, x);
        if (val <= 0.5) continue;
        img.at<cv::Vec3b>(y, x)[0] = img.at<cv::Vec3b>(y, x)[0] / 2 + bgr[0] / 2;
        img.at<cv::Vec3b>(y, x)[1] = img.at<cv::Vec3b>(y, x)[1] / 2 + bgr[1] / 2;
        img.at<cv::Vec3b>(y, x)[2] = img.at<cv::Vec3b>(y, x)[2] / 2 + bgr[2] / 2;
      }
    }

    cv::rectangle(img, r, bgr, 2);
    
    // Get the size of the text
    cv::Size textSize = cv::getTextSize(labels_map[(int)dets[i].class_id] + " " + to_string_with_precision(dets[i].conf), cv::FONT_HERSHEY_PLAIN, 1.2, 2, NULL);
    // Set the top left corner of the rectangle
    cv::Point topLeft(r.x, r.y - textSize.height);

    // Set the bottom right corner of the rectangle
    cv::Point bottomRight(r.x + textSize.width, r.y + textSize.height);

    // Set the thickness of the rectangle lines
    int lineThickness = 2;

    // Draw the rectangle on the image
    cv::rectangle(img, topLeft, bottomRight, bgr, -1);

    cv::putText(img, labels_map[(int)dets[i].class_id] + " " + to_string_with_precision(dets[i].conf), cv::Point(r.x, r.y + 4), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar::all(0xFF), 2);

  }
}

