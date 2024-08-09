//
// Created by aiden on 20-2-25.
//

#ifndef FSERVO_DCU_UTILS_H
#define FSERVO_DCU_UTILS_H
#include "fservo.h"
#include "typesdef.h"

using namespace snowlake;

/* *
 * @初始化dcu
 * @param[0]  cfg_path 配置文件路径
 * @param[1]  thread_num 计算线程的数量
 * return return_value_en
 * */
return_value_en dcu_init(const char *cfg_path, int thread_num);

/* *
 * @给dcu输入数据
 * @param[0] dcu_input, 输入数据,数据格式为(HWC)rgba
 * @param[1] call_back  输出的回调函数,该函数需要客户自己实现,并且该函数的消耗的时间也会包含在dcu的总消耗的时间中
 * return return_value_en
 * */
return_value_en dcu_input(dcu_input_st *dcu_input, call_back_func call_back);

/* *
 * @获取配置文件中配置参数
 * @param[0] section, 配置分类项
 * @param[1] name  配置名称
 * return return_value_en
 * */
template <class T>
return_value_en dcu_get_cfg_value(const cstring section, const cstring name, T &value);

/* *
 * @关闭dcu运行
 * @return void
 * */
void dcu_close();

/* *
 * @获取dcu的版本号
 * @return dcu_version_st
 * */
dcu_version_st dcu_get_version();

/* *
 * @获取dcu的状态值
 * @return dcu_status_st
 * */
dcu_status_st dcu_get_status();

#endif //FSERVO_DCU_UTILS_H
