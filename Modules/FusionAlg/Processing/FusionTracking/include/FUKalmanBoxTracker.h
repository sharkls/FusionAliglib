//
// Created by root on 10/24/22.
//

#ifndef TESTALGLIB_FUKALMANBOXTRACKER_H
#define TESTALGLIB_FUKALMANBOXTRACKER_H

#include <iostream>
#include <xtensor/xmath.hpp>
#include <cmath>
#include <string>
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <utility>
#include <xtensor/xmath.hpp>
#include <xtensor/xcontainer.hpp>
#include <xtensor/xsort.hpp>
#include <xtensor/xbuilder.hpp>
#include <vector>
#include <xtensor/xcsv.hpp>
#include <chrono>
#include "json.hpp"
#include "FUKalmanFilter.h"
#include "fusion_object.h"
using namespace std;
using namespace chrono;

struct state_output{
    xt::xarray<float> output_x;
    xt::xarray<float> bbox;
    float speed;
};

class FUKalmanBoxTracker{
public:

    FUKalmanBoxTracker(xt::xarray<float> bbox,unsigned long time_stamp, nlohmann::json &fusion_parameter, bool from_lidar = false,bool from_camera = false,bool from_radar = false);

    FUKalmanBoxTracker();

    FUKalmanBoxTracker(const FUKalmanBoxTracker & rhs);

    ~FUKalmanBoxTracker(){};

    // void update(xt::xarray<float> bbox,float dt);
    void updatewithlidar(xt::xarray<float> &bbox,unsigned long timestamp);
    void updatewithcamera(xt::xarray<float> &bbox,unsigned long timestamp);
    void updateonlywithcamera(xt::xarray<float> &bbox,unsigned long timestamp);
    void predict(float dt = 0.1);
    void update(unsigned long timestamp,float dt = 0.1);
    void update_onlyvideo(unsigned long timestamp,float dt = 0.1);

    static int _count;

private:

    void parse_json(nlohmann::json &fusion_parameter);
    void smooth_xy(float dt);
    void update_wlh();

public:

    int number = 0;
    unsigned long occur_time = 0;
    nlohmann::json m_fusion_parameter;
    int use_acc_model = 0;
    std::vector<int>  motor_vehicle_labels;
    std::vector<int>  motor_vehicle_labels_without_car;
    std::vector<int>  camera_create_trackers;
    float default_blind_area[4] = {0,0,5,5};
    FUKalmanFilter _kf;
    xt::xarray<float> _bbox;   // 一维向量

    //表示检测目标的ID
    int id = 0;
    int time_since_update = 0;
    std::vector <xt::xarray<float>> state;
    bool high_speed = false;
    bool low_speed = true;

    float lane_angle = -1000;
    float track_angle =-1000;
    float last_angle = -1000;
    float final_angle =-1000;
    bool track_angle_flag = false;
    bool first_true_angle_switch_flag = false;

    bool true_angle_switch_flag = false;
    std::vector<float> his_angle;

    float speed = 0;
    long int hits = 0;
    int label = 0;
    int lane = -1000;
    float speed_thresh = 3.0;
    xt::xarray<float> update_pixel_location = {-1000,-1000,-1000,-1000};
    float lefttopx = -1000;
    float lefttopy = -1000;
    float rightbottomx = -1000;
    float rightbottomy = -1000;
    TypeFusion _type_fusion;
    bool use_predict = false;
    xt::xarray<float>  predict_state;

    int lidar_updated = 0;
    int camera_updated = 0;
    int radar_updated = 0;
    unsigned long last_timestamp = 0;
    // 航向角修正标志位
    int angle_flag  = 0;
    int direction = 1;

    unsigned long lidar_timestamp;
    xt::xarray<float> last_lidar_state; // 一维向量

    unsigned long camera_timestamp;
    xt::xarray<float> last_camera_state; // 一维向量

    unsigned long radar_timestamp;
    xt::xarray<float> last_radar_state; // 一维向量

    xt::xarray<float> fusion_state; // 一维向量
    xt::xarray<float> last_fusion_state; // 一维向量
};

#endif //TESTALGLIB_FUKALMANBOXTRACKER_H