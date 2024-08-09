//
// Created by root on 10/24/22.
//
#include "FUKalmanBoxTracker.h"

FUKalmanBoxTracker::FUKalmanBoxTracker(xt::xarray<float> bbox, unsigned long time_stamp, nlohmann::json &fusion_parameter, bool from_lidar, bool from_camera, bool from_radar)
{
    // 解析融合的json参数
    m_fusion_parameter = fusion_parameter;
    parse_json(fusion_parameter);

    id = _count;
    _count += 1;
    occur_time = time_stamp;
    time_since_update = 0;
    use_acc_model = 0;

    if (int(bbox(7)) == 0 || int(bbox(7)) == 2 || int(bbox(7)) == 5 || int(bbox(7)) == 6 || int(bbox(7)) == 7 || int(bbox(7)) == 8)
    {
        use_acc_model = 1;
        _kf._model_type = 1; // 加速模型
        _kf = FUKalmanFilter(6, 2, 1);
        xt::xarray<float> bbox_init = xt::view(bbox, xt::range(0, 2));
        xt::view(_kf._x, xt::range(0, 2)) = bbox_init.reshape({bbox_init.size(), 1});
    }
    else
    {
        use_acc_model = 0; // 匀速模型
        _kf._model_type = 0;
        _kf = FUKalmanFilter(6, 2, 0);
        xt::xarray<float> bbox_init = xt::view(bbox, xt::range(0, 2));
        xt::view(_kf._x, xt::range(0, 2)) = bbox_init.reshape({bbox_init.size(), 1});
    }

    _bbox = bbox; // 一维向量
    high_speed = false;
    low_speed = true;

    lane_angle = -1000;
    track_angle = bbox(4);
    last_angle = bbox(4);
    final_angle = bbox(4);
    track_angle_flag = false;
    first_true_angle_switch_flag = false;

    speed = 0;
    hits = 0;
    label = int(bbox(7));
    lane = -1000;
    speed_thresh = 3.0;
    update_pixel_location = {-1000, -1000, -1000, -1000};
    lefttopx = -1000;
    lefttopy = -1000;
    rightbottomx = -1000;
    rightbottomy = -1000;
    float intersection_center[2] = {0, 0};
    _type_fusion = TypeFusion(bbox, fusion_parameter);
    use_predict = false;

    lidar_updated = 0;
    camera_updated = 0;
    radar_updated = 0;
    last_timestamp = time_stamp;
    // 航向角修正标志位
    angle_flag = 0;
    direction = 1;
}
FUKalmanBoxTracker::FUKalmanBoxTracker()
{
}
void FUKalmanBoxTracker::updatewithlidar(xt::xarray<float> &bbox, unsigned long timestamp)
{
    lidar_updated = 1;
    last_lidar_state = bbox;     //
    lidar_timestamp = timestamp; // 没啥用，可以考虑删除
    label = bbox(7);             // 更新类别,
    _type_fusion.updateType(bbox, 2);

}
void FUKalmanBoxTracker::updatewithcamera(xt::xarray<float> &bbox, unsigned long timestamp)
{
    camera_updated = 1;
    update_pixel_location = xt::view(bbox, xt::range(11, 15));
    _type_fusion.updateType(bbox, 1);
    last_camera_state = bbox;
    camera_timestamp = timestamp; // 没啥用，可以考虑删除
}
void FUKalmanBoxTracker::updateonlywithcamera(xt::xarray<float> &bbox, unsigned long timestamp)
{
    camera_updated = 1;
    update_pixel_location = xt::view(bbox, xt::range(11, 15));
    _type_fusion.updateType(bbox, 1);
    last_camera_state = bbox;
    label = bbox(7);
    camera_timestamp = timestamp; // 没啥用，可以考虑删除
}
void FUKalmanBoxTracker::predict(float dt)
{
    update_pixel_location = {10000, 10000, 10000, 10000};
    use_predict = true;
    use_acc_model = 0;
    _kf._model_type = 0;
    for (auto item : m_fusion_parameter["fusion_param"]["motor_vehicle_labels"].items())
    {
        if(_type_fusion.fusion_label == int(item.value())){
            use_acc_model = 1;
            _kf._model_type = 1;
        }
    }
    _kf.predict(dt, use_acc_model);

    lidar_updated = 0;
    camera_updated = 0;
    radar_updated = 0;
    predict_state = _kf._x;
    time_since_update += 1;
}
void FUKalmanBoxTracker::update(unsigned long timestamp, float dt)
{
    if (lidar_updated == 1 || camera_updated == 1 || radar_updated == 1)
    {
        if (lidar_updated == 1)
        {
            fusion_state = last_lidar_state;
        }
        else if (camera_updated == 1)
        {
            if (std::find(camera_create_trackers.begin(), camera_create_trackers.end(), _type_fusion.fusion_label) != camera_create_trackers.end() || m_fusion_parameter["fusion_param"]["camera_fill_blinds"])
            {
                if (m_fusion_parameter["fusion_param"]["camera_only_update_xy"]){
                    fusion_state(0) = last_camera_state(0);
                    fusion_state(1) = last_camera_state(1);
                }else{
                    fusion_state = last_camera_state;
                }
            }else{
                return;
            }

        }
        else
        {
            fusion_state = last_radar_state;
        }
        last_timestamp = timestamp;
        last_fusion_state = fusion_state;
        time_since_update = 0;
        hits += 1;
        smooth_xy(dt);

        xt::xarray<float> update_fusion_state = xt::view(fusion_state, xt::range(0, _kf._dim_z));
        update_fusion_state.reshape({1, update_fusion_state.size()});
        _kf.update(update_fusion_state);
        speed = std::sqrt(std::pow(_kf._x(2, 0), 2) + std::pow(_kf._x(3, 0), 2));
        update_wlh();
        _bbox = fusion_state;
        if (speed > speed_thresh)
        {
            high_speed = true;
            low_speed = false;
        }
        else
        {
            high_speed = false;
            low_speed = true;
        }
    }
}

void FUKalmanBoxTracker::update_onlyvideo(unsigned long timestamp, float dt)
{
    if(camera_updated)
    {
        fusion_state = last_camera_state;
        last_timestamp = timestamp;
        last_fusion_state = fusion_state;
        time_since_update = 0;
        hits += 1;
        smooth_xy(dt);

        xt::xarray<float> update_fusion_state = xt::view(fusion_state, xt::range(0, _kf._dim_z));
        update_fusion_state.reshape({1, update_fusion_state.size()});
        _kf.update(update_fusion_state);
        speed = std::sqrt(std::pow(_kf._x(2, 0), 2) + std::pow(_kf._x(3, 0), 2));
        update_wlh();
        _bbox = fusion_state;
        if (speed > speed_thresh)
        {
            high_speed = true;
            low_speed = false;
        }
        else
        {
            high_speed = false;
            low_speed = true;
        }
    }
}


void FUKalmanBoxTracker::parse_json(nlohmann::json &fusion_parameter)
{
    for (auto item : fusion_parameter["fusion_param"]["motor_vehicle_labels_without_car"].items())
    {
        motor_vehicle_labels_without_car.push_back(item.value());
    }

    for (auto item : fusion_parameter["fusion_param"]["motor_vehicle_labels"].items())
    {
        motor_vehicle_labels.push_back(item.value());
    }

    for (auto item : fusion_parameter["fusion_param"]["camera_create_trackers"].items())
    {
        camera_create_trackers.push_back(item.value());
    }
}

void FUKalmanBoxTracker::smooth_xy(float dt)
{
    float dt_ratio = dt / 0.1;
    direction = 1;
    int state_num = state.size();
    if (state_num > 3)
    {
        float dx_pre = state[state_num - 1](0) - state[state_num - 2](0);
        float dy_pre = state[state_num - 1](1) - state[state_num - 2](1);

        float dx = fusion_state(0) - state[state_num - 1](0);
        float dy = fusion_state(1) - state[state_num - 1](1);

        direction = (dx_pre * dx + dy_pre * dy) < 0 ? -1 : 1;
    }

    float vehicle_ratio;
    if (std::find(motor_vehicle_labels_without_car.begin(), motor_vehicle_labels_without_car.end(), _type_fusion.fusion_label) != motor_vehicle_labels_without_car.end())
    {
        vehicle_ratio = 0.9;
    }
    else
    {
        vehicle_ratio = 1;
    }

    if (std::find(motor_vehicle_labels.begin(), motor_vehicle_labels.end(), _type_fusion.fusion_label) != motor_vehicle_labels.end())
    {
        if (state_num > 5)
        {

            float pre_angle = (last_angle + 180) * M_PI / 180;
            float diff_x = fusion_state(0) - state[state_num - 1](0);
            float diff_y = fusion_state(1) - state[state_num - 1](1);
            float diff_len = sqrt(diff_x * diff_x + diff_y * diff_y);
            float diff_angle = atan2(diff_y, diff_x) * 180 / M_PI;
            diff_angle = (-1 * diff_angle + 90);
            diff_angle = std::fmod(diff_angle, 360.0);
            if (diff_angle < 0)
            {
                diff_angle = diff_angle + 360;
            }
            float new_angle = diff_angle * M_PI / 180;

            float d_angle = new_angle - pre_angle;

            float dis_lon = diff_len * cos(d_angle);
            float dis_lat = diff_len * sin(d_angle);

            // 计算纵向距离阈值
            float lon_thres;
            if (dis_lon < 0)
            {
                if (speed < 1.5)
                {
                    lon_thres = 0.85;
                }
                else if (speed >= 1.5 && speed < 3)
                {
                    lon_thres = 0.85;
                }
                else if (speed >= 3 && speed < 5)
                {
                    lon_thres = 0.85;
                }
                else if (speed >= 5 && speed < 10)
                {
                    lon_thres = 0.85;
                }
                else
                {
                    lon_thres = 0.1;
                }

                if (direction == 1)
                {
                    lon_thres = -1 * lon_thres;
                }
                else
                {
                    lon_thres = -0.5 * lon_thres;
                }
            }
            else
            {
                if (speed < 1.5)
                {
                    lon_thres = 0.5;
                }
                else if (speed >= 1.5 && speed < 3)
                {
                    lon_thres = 0.85;
                }
                else if (speed >= 3 && speed < 8)
                {
                    lon_thres = 0.85;
                }
                else if (speed >= 8 && speed < 10)
                {
                    lon_thres = speed * 0.1;
                }
                else
                {
                    lon_thres = speed * 0.1;
                }
            }

            // 计算横向距离阈值
            float lat_thres;
            if (lane_angle != -1000)
            {
                lat_thres = 0.35;
            }
            else
            {
                if (speed < 0.5)
                {
                    lat_thres = 0.3;
                }
                else if (speed >= 0.5 && speed < 1.5)
                {
                    lat_thres = 0.45;
                }
                else if (speed >= 1.5 && speed < 3)
                {
                    lat_thres = 0.45;
                }
                else if (speed >= 3 && speed < 5)
                {
                    lat_thres = 0.45;
                }
                else if (speed >= 5 && speed < 10)
                {
                    lat_thres = 0.45;
                }
                else
                {
                    lat_thres = 0.45;
                }
            }

            if (dis_lat < 0)
            {
                lat_thres = -1 * lat_thres;
            }

            if (dis_lat > 2)
            {
                dis_lat = 2;
            }

            if (dis_lat < -2)
            {
                dis_lat = -2;
            }

            lat_thres = lat_thres * vehicle_ratio * dt_ratio * 1.2;
            lon_thres = lon_thres * vehicle_ratio * dt_ratio * 1.2;

            // if(lat_thres > 0.9){
            //     lat_thres = 0.9;
            // }
            // if(lat_thres < -0.9){
            //     lat_thres = -0.9;
            // }

            d_angle = (d_angle * 180 / M_PI);
            d_angle = std::fmod(d_angle, 360.0);
            if(d_angle < 0){
                d_angle = d_angle +360.0;
            }

            float OA_x;
            float OA_y;

            if (d_angle > 90 && d_angle <= 270){
                OA_x =  abs(dis_lon) * sin(pre_angle + M_PI);
                OA_y =  abs(dis_lon) * cos(pre_angle + M_PI);
            }else{
                OA_x =  abs(dis_lon) * sin(pre_angle);
                OA_y =  abs(dis_lon) * cos(pre_angle);
            }

            float OB_x;
            float OB_y;

            if (d_angle > 0 && d_angle <= 180){
                OB_x = abs(dis_lat) * sin(pre_angle + M_PI / 2);
                OB_y = abs(dis_lat) * cos(pre_angle + M_PI / 2);
            }else{
                OB_x = abs(dis_lat) * sin(pre_angle -  M_PI / 2);
                OB_y = abs(dis_lat) * cos(pre_angle - M_PI / 2);
            }


            float dis_lon_reduct;
            float dis_lat_reduct;
            if (abs(dis_lon / lon_thres) > 1)
            {
                dis_lon_reduct = lon_thres * (1 + log10(dis_lon / lon_thres));
            }
            else
            {
                dis_lon_reduct = dis_lon;
            }

            if (abs(dis_lat / lat_thres) > 1)
            {
                dis_lat_reduct = lat_thres * (1 + log10(dis_lat / lat_thres));
            }
            else
            {
                dis_lat_reduct = dis_lat;
            }

            float  ratio_OB = abs(dis_lat_reduct / dis_lat);
            float  ratio_OA = abs(dis_lon_reduct / dis_lon);
            float dx =  ratio_OA * OA_x + ratio_OB * OB_x;
            float dy =  ratio_OA * OA_y + ratio_OB * OB_y;

            fusion_state(0) = state[state_num - 1](0) + dx;
            fusion_state(1) = state[state_num - 1](1) + dy;
        }
    }
}

void FUKalmanBoxTracker::update_wlh()
{
    float alpha;
    if (hits > 50){
        alpha = 1.0 / 50.0;
    }else{
        alpha = 1.0 / (float)hits;
    }
    fusion_state(2) = _bbox(2) + alpha * (fusion_state(2) - _bbox(2));
    fusion_state(3) = _bbox(3) + alpha * (fusion_state(3) - _bbox(3));
    fusion_state(6) = _bbox(6) + alpha * (fusion_state(6) - _bbox(6));

}

// 拷贝构造函数
FUKalmanBoxTracker::FUKalmanBoxTracker(const FUKalmanBoxTracker &rhs)
{

    number = rhs.number;
    occur_time = rhs.occur_time;
    m_fusion_parameter = rhs.m_fusion_parameter;
    use_acc_model = rhs.use_acc_model;
    motor_vehicle_labels = rhs.motor_vehicle_labels;
    motor_vehicle_labels_without_car = rhs.motor_vehicle_labels_without_car;
    camera_create_trackers = rhs.camera_create_trackers;
    default_blind_area[4] = rhs.default_blind_area[4];
    _kf = rhs._kf;
    _bbox = rhs._bbox;
    id = rhs.id;
    time_since_update = rhs.time_since_update;
    state = rhs.state;
    high_speed = rhs.high_speed;
    low_speed = rhs.low_speed;
    lane_angle = lane_angle;
    track_angle = rhs.track_angle;
    last_angle = rhs.last_angle;
    final_angle = rhs.final_angle;
    track_angle_flag = track_angle_flag;
    first_true_angle_switch_flag = first_true_angle_switch_flag;
    true_angle_switch_flag = rhs.true_angle_switch_flag;
    his_angle = rhs.his_angle;
    speed = rhs.speed;
    hits = rhs.hits;
    label = rhs.label;
    lane = rhs.lane;
    speed_thresh = rhs.speed_thresh;
    update_pixel_location = rhs.update_pixel_location;
    lefttopx = rhs.lefttopx;
    lefttopy = rhs.lefttopy;
    rightbottomx = rhs.rightbottomx;
    rightbottomy = rhs.rightbottomy;
    use_predict = rhs.use_predict;
    predict_state = rhs.predict_state;
    lidar_updated = rhs.lidar_updated;
    camera_updated = rhs.camera_updated;
    radar_updated = rhs.radar_updated;
    last_timestamp = rhs.last_timestamp;
    angle_flag = rhs.angle_flag;
    direction = rhs.direction;
    lidar_timestamp = rhs.lidar_timestamp;
    last_lidar_state = rhs.last_lidar_state;
    camera_timestamp = rhs.camera_timestamp;
    last_camera_state = rhs.last_camera_state;
    radar_timestamp = rhs.radar_timestamp;
    last_radar_state = rhs.last_radar_state;
    fusion_state = rhs.fusion_state;
    last_fusion_state = rhs.last_fusion_state;
    _type_fusion = rhs._type_fusion;
}