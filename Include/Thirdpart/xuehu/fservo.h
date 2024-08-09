//
// Created by ldd on 19-10-22.
//

#ifndef FSERVO_FSERVO_H
#define FSERVO_FSERVO_H

#include <cstdint>
#include <string>
#include <map>

#define MAX_SUPPORT_SIZE 608

//return value enum
typedef enum{
    //成功
    ret_normal = 0,
    //函数参数传递错误
    ret_func_err,
    //buffer已经存满数据，五空余的buffer可用
    ret_buf_full,
    //DCU未初始化
    ret_dcu_not_init,
    //DCU初始化失败
    ret_dcu_init_failed,
    //DCU配置文件路径是无效的
    ret_dcu_file_path_invalid,
    //DCU配置文件不能打开
    ret_dcu_cfg_file_open_failed,
    //DCU配置文件不能读取
    ret_dcu_cfg_file_read_failed,
    //DCU申请空间失败
    ret_dcu_malloc_failed,
    //其他错误
    ret_other_err
} return_value_en;

const std::map<return_value_en, std::string> return_value_description = {
        std::pair<return_value_en, std::string>(ret_normal,                   "normal return."),
        std::pair<return_value_en, std::string>(ret_func_err,                 "function param error"),
        std::pair<return_value_en, std::string>(ret_buf_full,                 "fpga buffer is full"),
        std::pair<return_value_en, std::string>(ret_dcu_not_init,             "dcu not initialize"),
        std::pair<return_value_en, std::string>(ret_dcu_init_failed,          "dcu initialized failed"),
        std::pair<return_value_en, std::string>(ret_dcu_file_path_invalid,    "the file path is invalid"),
        std::pair<return_value_en, std::string>(ret_dcu_cfg_file_open_failed, "can't open file"),
        std::pair<return_value_en, std::string>(ret_dcu_cfg_file_read_failed, "can't read file"),
        std::pair<return_value_en, std::string>(ret_dcu_malloc_failed,        "dcu malloc failed"),
        std::pair<return_value_en, std::string>(ret_other_err,                "unkonwn err"),
};

//DCU init status
typedef enum{
    //DCU未初始化
    dcu_init_not_init = 0,
    //DCU初始化成功
    dcu_init_success,
    //DCU初始化失败
    dcu_init_failed
}dcu_init_status_en;

//DCU status, include DCU output number, DCU remain number, DCU init status
typedef struct{
    //DCU已经计算了多少帧
    uint64_t outputCount;
    //剩余多少帧未计算
    uint64_t remain_number;
    //DCU初始化状态
    dcu_init_status_en status;
}dcu_status_st;

typedef struct{
    //fpga版本号
    std::string dcu_fpga_version;
    //fservo版本号
    std::string dcu_fservo_version;
}dcu_version_st;

//FPGA支持的图片分辨率
typedef enum {
    //608*608
    size_608 = 0,
    //416*416
    size_416 = 5,
    //320*320
    size_320
} picture_resolution_en;

typedef struct {
    int a[4];
    unsigned short b[8];
}InputData_st;

typedef enum
{
    INPUT_STATE_IDLE   = 0,
    INPUT_STATE_INITING,
    INPUT_STATE_INITED,
    INPUT_STATE_RUNNING,
    INPUT_STATE_FINISH,
}INPUT_STATE;

typedef struct{
    void *pData; //x,y,z,r(0~1)
    unsigned int   nDataSize;// 输入缓存状态，内部维护c
    int            input_state;// 输入缓存状态，内部维护
    uint64_t id;
}dcu_input_st;


typedef struct{
    void *pData;// 检测结果数据,x,y,z,w,l,h,r,label,score,dir和2个补齐数据
    unsigned int DataLen;

    void *pData1; // 输出数据1（备用）
    unsigned int Data1Len;

    void *pData2;// 输出数据2（备用）
    unsigned int Data2Len;

}dcu_output_st;




typedef struct {
    //输入数据(RGBA格式)
    unsigned char *in_data;
    //输入数据长度
    unsigned int in_data_len;
    //输入fpga图片尺寸
    picture_resolution_en picture_resolution;
    //图片标识,和输出对应,用户自己使用
    unsigned long long index;
    //图片标识,和输出对应,用户自己使用
    unsigned long long interior_index;
} FPGAInput;

typedef struct {
    float x, y, w, h;
} BoxInfo;

typedef struct {
    BoxInfo bbox;
    int classes;
    float *prob;
    float *mask;
    float objectness;
    int sort_class;
} DetectionInfo;

typedef struct {
    //输出结果
    DetectionInfo *det;
    // 有效框数
    int boxes_valid;
    //类别
    int classes;
} FPGAOutput;

/**
 * dcu输出回调函数,并且回调所消耗的时间,包含在dcu中fpga输出结果的后期处理中
 * */
//typedef void (*call_back_func)(const uint64_t , uint8_t*, const uint64_t);
//typedef void (*call_back_func)(void * , uint8_t*, const uint64_t);

typedef void (*call_back_func)(dcu_input_st *pIn , dcu_output_st *pOut);


typedef enum
{
    ERR_ZERO = 0,

    //无效路径
    ERR_INVALID_PATH,

    //缓冲区满
    ERR_BUFF_FULL

} ErrorCode;

#endif //FSERVO_FSERVO_H
