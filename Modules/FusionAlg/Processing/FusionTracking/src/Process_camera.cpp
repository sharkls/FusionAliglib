#include "Process_camera.h"

Process_camera::Process_camera(nlohmann::json &parameter, std::shared_ptr<fusion_match_out> match_out, std::shared_ptr<cal_dis_result> cal_dis_result)
{
    m_fusion_parameter = parameter;
    m_associate_out = match_out.get();
    m_cal_dis_result = cal_dis_result.get();
    camera_box_all_channel = xt::zeros<float>({0, 17});
}

Process_camera::~Process_camera()
{
}

void Process_camera::process_iou(std::shared_ptr<IouAssociate> iou_associate)
{
    if (int(camera_box.shape(0)) == 0)
    { // 无检测数据情况
        // 直接更该匹配结果
        int track_box_num = track_box.shape(0);
        m_associate_out->_cost_matrix = xt::empty<float>({0, 0});
        m_associate_out->_matched_indices = xt::empty<int>({0, 2});
        m_associate_out->_unmatched_detections_indices.clear(); // = xt::empty<int>({0, 0});
        std::vector<int> track_index(track_box_num);
        iota(track_index.begin(), track_index.end(), 0);
        m_associate_out->_unmatched_trackers_indices = track_index; // xt::arange<int>(0, track_box_num);
        m_associate_out->_unmatched_detections = xt::empty<float>({0, 0});
    }
    else if (int(track_box.shape(0)) == 0) // 无轨迹数据
    {
        // 直接更改匹配结果  xt::empty<int>({0, 2})
        int camera_box_num = camera_box.shape(0);
        m_associate_out->_cost_matrix = xt::empty<float>({0, 0});
        m_associate_out->_matched_indices = xt::empty<int>({0, 2});
        std::vector<int> camera_index(camera_box_num);
        iota(camera_index.begin(), camera_index.end(), 0);
        m_associate_out->_unmatched_detections_indices = camera_index; // xt::arange<int>(0, lidar_box.shape(0));
        m_associate_out->_unmatched_trackers_indices.clear();          // = xt::empty<int>({0, 0});
        m_associate_out->_unmatched_detections = camera_box;

    }
    else if (int(camera_box.shape(0)) != 0 && int(track_box.shape(0)) != 0)
    {
        iou_associate->pre_process_associate_data(&camera_box, &track_box, 0);
        xt::xarray<float> cost_iou;
        iou_associate->cal_cost_matrix(cost_iou);
        iou_associate->execute_match(cost_iou, m_associate_out);
        iou_associate->post_process_associate_data(m_associate_out, &camera_box);
    }
}

void Process_camera::process_dis(std::shared_ptr<DistanceAssociate> dis_associate)
{
    std::vector<int> matched_all_dets_first_index = m_associate_out->_matched_detections_indices;
    std::vector<int> matched_all_trks_first_index = m_associate_out->_matched_trackers_indices;
    std::vector<int> unmatched_all_dets_first_index = m_associate_out->_unmatched_detections_indices;
    std::vector<int> unmatched_all_trks_first_index = m_associate_out->_unmatched_trackers_indices;
    xt::xarray<float> dets_for_second = m_associate_out->_unmatched_detections;
    xt::xarray<float> trks_for_second = xt::empty<float>({0, 8});
    xt::xarray<int> matched_second_indices; // 声明第二阶段匹配索引
    xt::xarray<float> cost_matrix_second;   // 声明第二阶段损失矩阵
    if (int(unmatched_all_trks_first_index.size()) > 0)
    {
        trks_for_second = xt::zeros<float>({int(unmatched_all_trks_first_index.size()), int(track_box.shape(1))});
        for (int i = 0; i < int((unmatched_all_trks_first_index.size())); i++)
        {
            xt::view(trks_for_second, i, xt::all()) = xt::view(track_box, unmatched_all_trks_first_index[i], xt::all());
        }
    }
    if (int(trks_for_second.shape(0)) == 0)
    { // 第二阶段无轨迹数据
        matched_second_indices = xt::empty<int>({0, 2});
        std::vector<int> unmatched_detections_index_second(int(dets_for_second.shape(0)));
        iota(unmatched_detections_index_second.begin(), unmatched_detections_index_second.end(), 0);
        cost_matrix_second = xt::empty<float>({0, 0});
        m_associate_out->_cost_matrix = cost_matrix_second;
        m_associate_out->_unmatched_detections_indices = unmatched_detections_index_second;
    }
    if (int(dets_for_second.shape(0)) == 0)
    { // 第二阶段无检测数据
        matched_second_indices = xt::empty<int>({0, 2});
        std::vector<int> unmatched_trackers_index_second(int(trks_for_second.shape(0)));
        iota(unmatched_trackers_index_second.begin(), unmatched_trackers_index_second.end(), 0);
        cost_matrix_second = xt::empty<float>({0, 0});
        m_associate_out->_cost_matrix = cost_matrix_second;
    }
    if (int(trks_for_second.shape(0)) != 0 && int(dets_for_second.shape(0)) != 0)
    {
        dis_associate->pre_process_associate_data(&dets_for_second, &trks_for_second, 0);
        dis_associate->cal_cost_matrix(cost_matrix_second);
        dis_associate->execute_match(cost_matrix_second, m_associate_out);
        dis_associate->post_process_associate_data(m_associate_out, &dets_for_second);
    }
}

void Process_camera::set_out()
{
    std::vector<int> matched_all_trks_id;   // 存储所有匹配上的目标轨迹id
    for(int i = 0; i< m_associate_out->_matched_trackers_indices.size(); i++){
        matched_all_trks_id.push_back(int(track_box(m_associate_out->_matched_trackers_indices[i], 7)));
    }
    m_associate_out->_matched_all_trks_id = matched_all_trks_id;

    // 把和轨迹匹配上的相机检测结果id改成轨迹id
    int matched_num = m_associate_out->_matched_detections_indices.size();
    for (int i = 0; i < matched_num; i++){
        int box_index = m_associate_out->_matched_detections_indices[i];
        camera_box(box_index, 9) = matched_all_trks_id[i];
    }
}

void Process_camera::process_camera_data(xt::xarray<float> *_p_pCameraResult)
{
    xt::xarray<float> empty_data = xt::empty<float>({0, 0});
    // 先清除上一帧match_out的数据
    m_associate_out->_cost_matrix = empty_data;
    m_associate_out->_matched_indices = empty_data;
    m_associate_out->_matched_detections_indices.clear();
    m_associate_out->_matched_trackers_indices.clear();
    m_associate_out->_unmatched_detections_indices.clear();
    m_associate_out->_unmatched_trackers_indices.clear();
    m_associate_out->_unmatched_detections = empty_data;
    m_associate_out->_unmatched_trackers = empty_data;
    m_associate_out->_matched_all_trks_id.clear();

    m_cal_dis_result->ano_dis = empty_data;
    m_cal_dis_result->head_dis = empty_data;
    m_cal_dis_result->square_dis = empty_data;
    m_cal_dis_result->ano_dis_thres_matrix = empty_data;
    m_cal_dis_result->head_dis_thres_matrix = empty_data;
    camera_box = xt::zeros<float>({0, 17});
    camera_box = *_p_pCameraResult;
}
void Process_camera::excute_match_camera_stage(std::vector<FUKalmanBoxTracker> &_trackers, std::shared_ptr<IouAssociate> iou_associate, std::shared_ptr<DistanceAssociate> dis_associate)
{
    track_box = xt::empty<float>({0, 8});
    int trackers_num = _trackers.size();
    track_box = xt::zeros<float>({trackers_num, 8});
    std::vector<int> invaild_trackers_index;
    for (int i = 0; i < trackers_num; i++)
    {
        track_box(i, 0) = _trackers[i]._kf._x(0, 0); // x
        track_box(i, 1) = _trackers[i]._kf._x(1, 0); // y
        track_box(i, 2) = _trackers[i]._bbox(5);     // z
        track_box(i, 3) = _trackers[i]._bbox(2);     // w
        track_box(i, 4) = _trackers[i]._bbox(3);     // l
        track_box(i, 5) = _trackers[i]._bbox(6);     // h
        float heading_angle;
        if (_trackers[i].last_angle != None && _trackers[i].last_angle != -1000)
        {
            heading_angle = _trackers[i].last_angle;
        }
        else
        {
            heading_angle = _trackers[i]._bbox(4); // angle
        }
        track_box(i, 6) = heading_angle;      // theta
        track_box(i, 7) = _trackers[i].id + 1; // id
        if (xt::any(xt::isnan(_trackers[i]._kf._x)))
            invaild_trackers_index.push_back(i);
    }
    // 去除无效的轨迹数据
    if (int(invaild_trackers_index.size()) > 0)
    {
        track_box = xt::view(track_box, xt::drop(invaild_trackers_index));
    }
    process_iou(iou_associate);
    process_dis(dis_associate);
    set_out();
}

void Process_camera::update_type(std::vector<FUKalmanBoxTracker> &_trackers, unsigned long timestamp)
{
    int trackers_num = _trackers.size();

    for (int i = 0; i < trackers_num; i++)
    {
        if (std::find(m_associate_out->_unmatched_trackers_indices.begin(), m_associate_out->_unmatched_trackers_indices.end(), i) == m_associate_out->_unmatched_trackers_indices.end())
        { // 表示i不在_unmatched_trackers_indices中,即检测和轨迹匹配成功了，就更新轨迹
            auto detection_indices = xt::where(xt::col(m_associate_out->_matched_indices, 1) > i - 1 && xt::col(m_associate_out->_matched_indices, 1) < i + 1)[0];
            if (int(detection_indices.size()) > 0)
            {
                int index = detection_indices[0];
                xt::xarray<float> bbox = xt::view(camera_box, int(m_associate_out->_matched_indices(index, 0)), xt::all());
                _trackers[i].updatewithcamera(bbox, timestamp);
            }
        }
    }
    camera_box_all_channel = xt::vstack(xt::xtuple(camera_box_all_channel, camera_box));
}

void Process_camera::update_onlyvideo(std::vector<FUKalmanBoxTracker> &_trackers, unsigned long timestamp)
{
    int trackers_num = _trackers.size();
    for (int i = 0; i < trackers_num; i++)
    {
        if (std::find(m_associate_out->_unmatched_trackers_indices.begin(), m_associate_out->_unmatched_trackers_indices.end(), i) == m_associate_out->_unmatched_trackers_indices.end())
        { // 表示i不在_unmatched_trackers_indices中,即检测和轨迹匹配成功了，就更新轨迹
            auto detection_indices = xt::where(xt::col(m_associate_out->_matched_indices, 1) > i - 1 && xt::col(m_associate_out->_matched_indices, 1) < i + 1)[0];
            if (int(detection_indices.size()) > 0)
            {
                int index = detection_indices[0];
                xt::xarray<float> bbox = xt::view(camera_box, int(m_associate_out->_matched_indices(index, 0)), xt::all());
                _trackers[i].updateonlywithcamera(bbox, timestamp);
            }
        }
    }
    camera_box_all_channel = xt::vstack(xt::xtuple(camera_box_all_channel, camera_box));
}

void Process_camera::getCameraBoxInfo(std::vector<xt::xarray<float>> *_p_pCameraResult)
{
    int nVideoCount = _p_pCameraResult->size();
    // 直接发送相机原始检测结果
    if (m_fusion_parameter["fusion_param"]["video_for_show_flag"]){
        for (int i = 0; i < nVideoCount; i++){

            xt::xarray<float> current_box = (*_p_pCameraResult)[i];
            _p_pCameraResult->at(i) = current_box;
        }
    }else{  // 给相机的检测结果加上匹配结果
        for (int i = 0; i < nVideoCount; i++){
            int index = i+1;
            auto reverse_index = xt::where(xt::col(camera_box_all_channel, 16) >= index && xt::col(camera_box_all_channel, 16) <= index)[0];
            xt::xarray<float> video_for_fusion_single_channel = xt::view(camera_box_all_channel, xt::keep(reverse_index));
            auto matched_index = xt::where(xt::col(video_for_fusion_single_channel, 9) > 0)[0];
            xt::view(video_for_fusion_single_channel, xt::keep(matched_index), 15) = 2; // 2: 融合
            _p_pCameraResult->at(i) = video_for_fusion_single_channel;
        }
    }
    camera_box_all_channel = xt::zeros<float>({0, 17});
}
