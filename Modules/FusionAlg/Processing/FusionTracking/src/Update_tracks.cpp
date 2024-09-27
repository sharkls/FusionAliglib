#include <Update_tracks.h>

// Update_tracks::Update_tracks(nlohmann::json &parameter, std::unordered_map<int, std::pair<int, float>> *chedao, float x_limit[], float y_limit[])
// {
//     m_fusion_parameter = parameter;
//     // m_chedao = chedao;
//     // m_height = int((y_limit[1] - y_limit[0]) * 10);
//     // m_width = int((x_limit[1] - x_limit[0]) * 10);
//     // m_x_limit = x_limit;
//     // m_y_limit = y_limit;
//     parse_json();
// }

Update_tracks::Update_tracks(nlohmann::json &parameter)
{
    m_fusion_parameter = parameter;
    // m_chedao = chedao;
    // m_height = int((y_limit[1] - y_limit[0]) * 10);
    // m_width = int((x_limit[1] - x_limit[0]) * 10);
    // m_x_limit = x_limit;
    // m_y_limit = y_limit;
    parse_json();

}

Update_tracks::~Update_tracks()
{
}

void Update_tracks::get_state(FUKalmanBoxTracker& tracker)
{
    xt::xarray<float> current_state = {tracker._kf._x(0, 0), tracker._kf._x(1, 0), tracker.speed};
    tracker.state.push_back(current_state);

    std::vector<xt::xarray<float>> last_thirty_state;
    if (int(tracker.state.size()) > 30)
    {
        last_thirty_state.reserve(30);
        std::copy(tracker.state.end() - 30, tracker.state.end(), std::back_inserter(last_thirty_state));
        tracker.state = last_thirty_state;
    }
}

void Update_tracks::get_angle(FUKalmanBoxTracker& tracker)
{
    // 删除根据车道号获取目标角度的代码
    // std::pair<int, float> lane_lane_angle;
    // lane_lane_angle = FunctionHub::get_lane_info(tracker._kf._x(0, 0), tracker._kf._x(1, 0), m_width, m_height, m_x_limit, m_y_limit, m_chedao);
    
    // tracker.lane = lane_lane_angle.first;
    // tracker.lane_angle = lane_lane_angle.second;

    std::pair<float, bool> angle_and_angle_flag = FusionFunction::cal_angle_new(&tracker, m_fusion_parameter);
    tracker.track_angle = angle_and_angle_flag.first;
    tracker.track_angle_flag = angle_and_angle_flag.second;

    if (!tracker.track_angle_flag && tracker.last_angle != -1000.0){
        tracker.track_angle = tracker.last_angle;
    }
    
    // 删除根据车道角度更新跟踪角度的代码
    // if (m_fusion_parameter["fusion_param"]["flag_lane_angle2track_angle"] &&
    //     tracker.lane_angle != -1000 && std::find(motor_vehicle_labels.begin(), motor_vehicle_labels.end(), tracker._type_fusion.fusion_label) != motor_vehicle_labels.end())
    // {
    //     tracker.track_angle = tracker.lane_angle;
    // }

    if (m_fusion_parameter["fusion_param"]["flag_smooth_angle"] ){
        FusionFunction::smooth_angle(&tracker, &motor_vehicle_labels);
    }

    // 保存最新的30帧的航向角
    std::vector<float> last_thirty_angle;
    if (int(tracker.his_angle.size()) > 30){
        tracker.his_angle.erase(tracker.his_angle.begin(), tracker.his_angle.begin() + tracker.his_angle.size() - 30);
    }

    tracker.final_angle = tracker.track_angle;
    tracker.his_angle.push_back(tracker.final_angle);
    tracker.last_angle = tracker.final_angle;
}


void Update_tracks::create_tracks(std::shared_ptr<fusion_match_out> _match_out, std::vector<FUKalmanBoxTracker> &_m_trackers, unsigned long timestamp, int flag)
{
    auto unmatched_detections = _match_out->_unmatched_detections;
    int unmatched_dets_num = unmatched_detections.shape(0);
    if (unmatched_dets_num > 0)
    {
        for (int i = 0; i < unmatched_dets_num; i++)
        {
            xt::xarray<float> bbox = xt::view(unmatched_detections, i, xt::all());
            if (flag == 1)
            { // 雷达  根据车道获取点云目标的航向角
                // float lane_angle = FunctionHub::get_lane_info(bbox(0), bbox(1), m_width, m_height, m_x_limit, m_y_limit, m_chedao).second;
                // if (lane_angle != -1000)
                // {
                //     bbox(4) = lane_angle;
                // }
                _m_trackers.push_back(FUKalmanBoxTracker(bbox, timestamp, m_fusion_parameter, true, false, false));
            }

            // if (flag == 0)
            // { // 相机 label:bbox(7)
            //     if (std::find(camera_create_trackers.begin(), camera_create_trackers.end(), int(bbox(7))) != camera_create_trackers.end())
            //     {
            //         float lane_angle = FunctionHub::get_lane_info(bbox(0), bbox(1), m_width, m_height, m_x_limit, m_y_limit, m_chedao).second;
            //         if (lane_angle != -1000)
            //         {
            //             bbox(4) = lane_angle;
            //         }
            //         _m_trackers.push_back(FUKalmanBoxTracker(bbox, timestamp, m_fusion_parameter, false, true, false));
            //     }
            // }
        }
    }
}

// void Update_tracks::create_onlyvideotracks(std::shared_ptr<fusion_match_out> _match_out, std::vector<FUKalmanBoxTracker> &_m_trackers, unsigned long timestamp, int flag)
// {
//     auto unmatched_detections = _match_out->_unmatched_detections;
//     int unmatched_dets_num = unmatched_detections.shape(0);
//     if (unmatched_dets_num > 0)
//     {
//         for (int i = 0; i < unmatched_dets_num; i++)
//         {
//             xt::xarray<float> bbox = xt::view(unmatched_detections, i, xt::all());
//             float lane_angle = FunctionHub::get_lane_info(bbox(0), bbox(1), m_width, m_height, m_x_limit, m_y_limit, m_chedao).second;
//             if (lane_angle != -1000)
//             {
//                 bbox(4) = lane_angle;
//             }
//             _m_trackers.push_back(FUKalmanBoxTracker(bbox, timestamp, m_fusion_parameter, false, true, false));
//         }
//     }
// }

void Update_tracks::update(std::vector<FUKalmanBoxTracker> &_m_trackers, xt::xarray<float> &res, unsigned long timestamp, float dt)
{
    int trackers_num = _m_trackers.size();
    std::vector<int> pop_tracker_index;  // 删除轨迹的索引
    std::vector<long int> all_hits;
    for (int i = 0; i < trackers_num; i++)
    {
        if ((_m_trackers[i].time_since_update > int(m_fusion_parameter["fusion_param"]["sort_max_age_new"]))) //   || ((_m_trackers[i].time_since_update > _m_trackers[i].hits) && (_m_trackers[i].time_since_update>2) )
        {
            pop_tracker_index.push_back(i);
        }
        _m_trackers[i].update(timestamp, dt);
        get_state(_m_trackers[i]);
        get_angle(_m_trackers[i]);

        all_hits.push_back(_m_trackers[i].hits);

        int src = 0;
        if (_m_trackers[i].camera_updated == 1 && _m_trackers[i].lidar_updated  == 0 && _m_trackers[i].radar_updated == 0){
            src = 1;
        }else if (_m_trackers[i].camera_updated == 0 && _m_trackers[i].lidar_updated  == 1 && _m_trackers[i].radar_updated == 0)
        {
            src = 2;
        }else if (_m_trackers[i].camera_updated == 0 && _m_trackers[i].lidar_updated  == 0 && _m_trackers[i].radar_updated == 1){
            src = 3;
        }else if(_m_trackers[i].camera_updated == 1 && _m_trackers[i].lidar_updated  == 1 && _m_trackers[i].radar_updated == 0){
            src = 4;
        }else if (_m_trackers[i].camera_updated == 0 && _m_trackers[i].lidar_updated  == 1 && _m_trackers[i].radar_updated == 1){
            src = 5;
        }else if (_m_trackers[i].camera_updated == 1 && _m_trackers[i].lidar_updated  == 0 && _m_trackers[i].radar_updated == 1){
            src = 6;
        }else if (_m_trackers[i].camera_updated == 1 && _m_trackers[i].lidar_updated  == 1 && _m_trackers[i].radar_updated == 1){
            src = 7;
        }

        // bbox:shape:(n, 1)
        // _kf._x: shape:(n, 1)
        // 融合结果输出数据： [x, y, w, l, theta, z, h, label, speed, id, scores, x_min, y_min, x_max, y_max, data_source, channel, lane,occur_time, time_sicne_update, hits, flag_delete_track]
        if(_m_trackers[i]._kf._x(0, 0) > m_fusion_parameter["fusion_param"]["out_xmin"] &&
           _m_trackers[i]._kf._x(0, 0) < m_fusion_parameter["fusion_param"]["out_xmax"] &&
           _m_trackers[i]._kf._x(1, 0) > m_fusion_parameter["fusion_param"]["out_ymin"] &&
           _m_trackers[i]._kf._x(1, 0) < m_fusion_parameter["fusion_param"]["out_ymax"] &&
           _m_trackers[i]._bbox(5) > m_fusion_parameter["fusion_param"]["out_zmin"] &&
           _m_trackers[i]._bbox(5) < m_fusion_parameter["fusion_param"]["out_zmax"] )
        {
            xt::xarray<float> conv = {
                    _m_trackers[i]._kf._x(0, 0),   // 0: x
                    _m_trackers[i]._kf._x(1, 0),   // 1: y
                    _m_trackers[i]._bbox(2),     // 2: w  bbox:shape:(n, 1)
                    _m_trackers[i]._bbox(3),     // 3: l
                    _m_trackers[i].final_angle, // 4: angle
                    _m_trackers[i]._bbox(5),     // 5: z
                    _m_trackers[i]._bbox(6),      // 6: h
                    float(_m_trackers[i]._type_fusion.fusion_label),         // 7: label
                    _m_trackers[i].speed,                  // 8: speed
                    float(_m_trackers[i].id + 1), // 9: id
                    float(_m_trackers[i]._type_fusion.fusion_score),                  // 10: score
                    _m_trackers[i].update_pixel_location(0),                  // 11: x_min
                    _m_trackers[i].update_pixel_location(1),                  // 12: y_min
                    _m_trackers[i].update_pixel_location(2),                  // 13: x_max
                    _m_trackers[i].update_pixel_location(3),                  // 14: y_max
                    float(src),                  // 15: data_source
                    0,                  // 16: channel
                    float(_m_trackers[i].lane),                  // 17: lane
                    float(_m_trackers[i].occur_time),                  // 18: occur_time
                    float(_m_trackers[i].time_since_update),  // 19: time_since_update
                    float(_m_trackers[i].hits),               // 20: hits
                    0                                         // 21: delete track flag

            };
            conv.reshape({1, conv.size()});
            res = xt::concatenate(xt::xtuple(res, conv), 0);
        }

    }
    if (pop_tracker_index.size() > 0){
        for (int i = int(pop_tracker_index.size()-1); i>=0; i--)
        {
            int index = pop_tracker_index[i];
            _m_trackers.erase(_m_trackers.begin()+index);
        }
    }
    std::cout << "---------------Update_tracks : fusion_track_result_nums: " << res.shape(0) << std::endl;
}

// void Update_tracks::update_onlyvideo(std::vector<FUKalmanBoxTracker> &_m_trackers, xt::xarray<float> &res, unsigned long timestamp, float dt)
// {
//     int trackers_num = _m_trackers.size();
//     std::vector<int> pop_tracker_index;  // 删除轨迹的索引
//     std::vector<long int> all_hits;
//     for (int i = 0; i < trackers_num; i++)
//     {
//         _m_trackers[i].update_onlyvideo(timestamp, dt);
//         get_state(_m_trackers[i]);
//         get_angle(_m_trackers[i]);
//         if ((_m_trackers[i].time_since_update > int(m_fusion_parameter["fusion_param"]["sort_max_age_new"]))) //   || ((_m_trackers[i].time_since_update > _m_trackers[i].hits) && (_m_trackers[i].time_since_update>2) )
//         {
//             pop_tracker_index.push_back(i);
//         }
//         all_hits.push_back(_m_trackers[i].hits);
//         int src = 0;
//         if (_m_trackers[i].camera_updated == 1 && _m_trackers[i].lidar_updated  == 0 && _m_trackers[i].radar_updated == 0){
//             src = 1;
//         }else if (_m_trackers[i].camera_updated == 0 && _m_trackers[i].lidar_updated  == 1 && _m_trackers[i].radar_updated == 0)
//         {
//             src = 2;
//         }else if (_m_trackers[i].camera_updated == 0 && _m_trackers[i].lidar_updated  == 0 && _m_trackers[i].radar_updated == 1){
//             src = 3;
//         }else if(_m_trackers[i].camera_updated == 1 && _m_trackers[i].lidar_updated  == 1 && _m_trackers[i].radar_updated == 0){
//             src = 4;
//         }else if (_m_trackers[i].camera_updated == 0 && _m_trackers[i].lidar_updated  == 1 && _m_trackers[i].radar_updated == 1){
//             src = 5;
//         }else if (_m_trackers[i].camera_updated == 1 && _m_trackers[i].lidar_updated  == 0 && _m_trackers[i].radar_updated == 1){
//             src = 6;
//         }else if (_m_trackers[i].camera_updated == 1 && _m_trackers[i].lidar_updated  == 1 && _m_trackers[i].radar_updated == 1){
//             src = 7;
//         }
//         // bbox:shape:(n, 1)
//         // _kf._x: shape:(n, 1)
//         // 融合结果输出数据： [x, y, w, l, theta, z, h, label, speed, id, scores, x_min, y_min, x_max, y_max, data_source, channel, lane,occur_time, time_sicne_update, hits, flag_delete_track]
//         if(_m_trackers[i]._kf._x(0, 0) > m_fusion_parameter["fusion_param"]["out_xmin"] &&
//            _m_trackers[i]._kf._x(0, 0) < m_fusion_parameter["fusion_param"]["out_xmax"] &&
//            _m_trackers[i]._kf._x(1, 0) > m_fusion_parameter["fusion_param"]["out_ymin"] &&
//            _m_trackers[i]._kf._x(1, 0) < m_fusion_parameter["fusion_param"]["out_ymax"] &&
//            _m_trackers[i]._bbox(5) > m_fusion_parameter["fusion_param"]["out_zmin"] &&
//            _m_trackers[i]._bbox(5) < m_fusion_parameter["fusion_param"]["out_zmax"] ){
//             xt::xarray<float> conv = {
//                 _m_trackers[i]._kf._x(0, 0),   // 0: x
//                 _m_trackers[i]._kf._x(1, 0),   // 1: y
//                 _m_trackers[i]._bbox(2),     // 2: w  bbox:shape:(n, 1)
//                 _m_trackers[i]._bbox(3),     // 3: l
//                 _m_trackers[i].final_angle, // 4: angle
//                 _m_trackers[i]._bbox(5),     // 5: z
//                 _m_trackers[i]._bbox(6),      // 6: h
//                 float(_m_trackers[i]._type_fusion.fusion_label),         // 7: label
//                 _m_trackers[i].speed,                  // 8: speed
//                 float(_m_trackers[i].id + 1), // 9: id
//                 float(_m_trackers[i]._type_fusion.fusion_score),                  // 10: score
//                 _m_trackers[i].update_pixel_location(0),                  // 11: x_min
//                 _m_trackers[i].update_pixel_location(1),                  // 12: y_min
//                 _m_trackers[i].update_pixel_location(2),                  // 13: x_max
//                 _m_trackers[i].update_pixel_location(3),                  // 14: y_max
//                 float(src),                  // 15: data_source
//                 0,                  // 16: channel
//                 float(_m_trackers[i].lane),                  // 17: lane
//                 float(_m_trackers[i].occur_time),                  // 18: occur_time
//                 float(_m_trackers[i].time_since_update),  // 19: time_since_update
//                 float(_m_trackers[i].hits),               // 20: hits
//                 0                                         // 21: delete track flag
//             };
//             conv.reshape({1, conv.size()});
//             res = xt::concatenate(xt::xtuple(res, conv), 0);
//         }
//     }
//     // 删除轨迹
//     if (pop_tracker_index.size() > 0){
//         for (int i = int(pop_tracker_index.size()-1); i>=0; i--)
//         {
//             int index = pop_tracker_index[i];
//             _m_trackers.erase(_m_trackers.begin()+index);
//         }
//     }
//     _m_trackers.shrink_to_fit();
// }

void Update_tracks::parse_json()
{
    for (auto item : m_fusion_parameter["fusion_param"]["camera_create_trackers"].items())
    {
        camera_create_trackers.push_back(item.value());
    }

    for (auto item : m_fusion_parameter["fusion_param"]["motor_vehicle_labels"].items())
    {
        motor_vehicle_labels.push_back(item.value());
    }
}