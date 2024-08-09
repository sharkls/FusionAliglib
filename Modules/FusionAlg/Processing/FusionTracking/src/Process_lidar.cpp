#include <Process_lidar.h>
#include "FunctionHub.h"

Process_lidar::Process_lidar(nlohmann::json &parameter, std::shared_ptr<fusion_match_out> match_out, std::shared_ptr<cal_dis_result> cal_dis_result)
{
    m_fusion_parameter = parameter;
    m_associate_out = match_out.get();
    m_cal_dis_result = cal_dis_result.get();
}
Process_lidar::~Process_lidar()
{
}

void Process_lidar::setout_det(std::vector<int>& matched_all_dets_index, std::vector<int>& unmatched_elder_trks_index, std::vector<int>& matched_elder_trks_index)
{
    m_associate_out->_cost_matrix = xt::empty<float>({0, 0});
    if (int(matched_all_dets_index.size()) <= 0){
        m_associate_out->_matched_indices = xt::empty<int>({0, 2});
    }
    else{
        std::vector<int> matched_all_trks_index;
        for(int i = 0; i < int((matched_elder_trks_index.size())); i++){
            matched_all_trks_index.push_back(vaild_elder[matched_elder_trks_index[i]]);
        }
        assert(matched_all_dets_index.size() == matched_all_trks_index.size());
        std::vector<std::size_t> shape = {matched_all_dets_index.size(), 1};
        xt::xarray<int> xt_matched_det_index = xt::adapt(matched_all_dets_index, shape);
        xt::xarray<int> xt_matched_trk_index = xt::adapt(matched_all_trks_index, shape);
        m_associate_out->_matched_indices = xt::concatenate(xt::xtuple(xt_matched_det_index, xt_matched_trk_index), 1);
        m_associate_out->_matched_trackers_indices = matched_all_trks_index;
    }

    m_associate_out->_unmatched_detections_indices.clear();

    std::vector<int> track_index;
    for(int i = 0; i < int((unmatched_elder_trks_index.size())); i++){
        track_index.push_back(vaild_elder[unmatched_elder_trks_index[i]]);
    }
    track_index.insert(track_index.end(), vaild_younger.begin(), vaild_younger.end());
    m_associate_out->_unmatched_trackers_indices = track_index;
    m_associate_out->_unmatched_detections = xt::empty<float>({0, 0});
}

void Process_lidar::setout_trk(std::vector<int>& matched_all_dets_index, std::vector<int>& unmatched_elder_trks_index, std::vector<int>& matched_elder_trks_index)
{
    m_associate_out->_cost_matrix = xt::empty<float>({0, 0});
    if (int(matched_all_dets_index.size()) <= 0){
        m_associate_out->_matched_indices = xt::empty<int>({0, 2});
    }
    else{
        std::vector<int> matched_all_trks_index;
        for(int i = 0; i < int((matched_elder_trks_index.size())); i++){
            matched_all_trks_index.push_back(vaild_elder[matched_elder_trks_index[i]]);
        }
        assert(matched_all_dets_index.size() == matched_all_trks_index.size());
        std::vector<std::size_t> shape = {matched_all_dets_index.size(), 1};
        xt::xarray<int> xt_matched_det_index = xt::adapt(matched_all_dets_index, shape);
        xt::xarray<int> xt_matched_trk_index = xt::adapt(matched_all_trks_index, shape);
        m_associate_out->_matched_indices = xt::concatenate(xt::xtuple(xt_matched_det_index, xt_matched_trk_index), 1);
        m_associate_out->_matched_trackers_indices = matched_all_trks_index;
    }
    std::vector<int> track_index;
    for(int i = 0; i < int((unmatched_elder_trks_index.size())); i++){
        track_index.push_back(vaild_elder[unmatched_elder_trks_index[i]]);
    }
    m_associate_out->_unmatched_trackers_indices = track_index;
}

void Process_lidar::process_iou(std::shared_ptr<IouAssociate> iou_associate)
{
    if (int(lidar_box.shape(0)) == 0)
    {   // 直接更该匹配结果
        int track_box_num = elder_track_box.shape(0);
        m_associate_out->_cost_matrix = xt::empty<float>({0, 0});
        m_associate_out->_matched_indices = xt::empty<int>({0, 2});
        m_associate_out->_unmatched_detections_indices.clear(); // = xt::empty<int>({0, 0});
        std::vector<int> track_index(track_box_num);
        iota(track_index.begin(), track_index.end(), 0);
        m_associate_out->_unmatched_trackers_indices = track_index; //xt::arange<int>(0, track_box_num);
        m_associate_out->_unmatched_detections = xt::empty<float>({0, 0});
    }
    if (int(elder_track_box.shape(0)) == 0) // 无轨迹数据
    {
        // 直接更改匹配结果  xt::empty<int>({0, 2})
        int lidar_box_num = lidar_box.shape(0);
        m_associate_out->_cost_matrix = xt::empty<float>({0, 0});
        m_associate_out->_matched_indices = xt::empty<int>({0, 2});
        std::vector<int> lidar_index(lidar_box_num);
        iota(lidar_index.begin(), lidar_index.end(), 0);
        m_associate_out->_unmatched_detections_indices = lidar_index; //xt::arange<int>(0, lidar_box.shape(0));
        m_associate_out->_unmatched_trackers_indices.clear(); // = xt::empty<int>({0, 0});
        m_associate_out->_unmatched_detections = lidar_box;
    }
    if (int(lidar_box.shape(0)) != 0 && int(elder_track_box.shape(0)) != 0)
    {
        iou_associate->pre_process_associate_data(&lidar_box, &elder_track_box, 1);
        xt::xarray<float> cost_iou;
        iou_associate->cal_cost_matrix(cost_iou);
        iou_associate->execute_match(cost_iou, m_associate_out);
        iou_associate->post_process_associate_data(m_associate_out, &lidar_box);
    }
}

void Process_lidar::process_second(std::shared_ptr<DistanceAssociate> dis_associate)
{
    std::vector<int> unmatched_elder_trks_first_index = m_associate_out->_unmatched_trackers_indices;
    xt::xarray<float> dets_for_second = m_associate_out->_unmatched_detections;
    xt::xarray<float> elder_trks_for_second;
    if (int(unmatched_elder_trks_first_index.size()) > 0){
        elder_trks_for_second = xt::zeros<float>({int(unmatched_elder_trks_first_index.size()), int(elder_track_box.shape(1))});
        for(int i = 0; i < int((unmatched_elder_trks_first_index.size())); i++){
            xt::view(elder_trks_for_second, i, xt::all()) = xt::view(elder_track_box, unmatched_elder_trks_first_index[i], xt::all());
        }
    }else{
        elder_trks_for_second = xt::empty<float>({0, 8});
    }
    xt::xarray<float> cost_matrix_second;
    if(int(elder_trks_for_second.shape(0)) == 0){   // 第二阶段无轨迹数据
        cost_matrix_second = xt::empty<float>({0, 0});
        m_associate_out->_cost_matrix = cost_matrix_second;
    }
    if (int(dets_for_second.shape(0)) == 0){
        cost_matrix_second = xt::empty<float>({0, 0});
        m_associate_out->_cost_matrix = cost_matrix_second;
    }

    if(int(elder_trks_for_second.shape(0)) != 0 && int(dets_for_second.shape(0)) != 0){
        dis_associate->pre_process_associate_data(&dets_for_second, &elder_trks_for_second, 1);
        dis_associate->cal_cost_matrix(cost_matrix_second);
        dis_associate->execute_match(cost_matrix_second, m_associate_out);
        dis_associate->post_process_associate_data(m_associate_out, &dets_for_second);
    }
}

void Process_lidar::process_third(std::shared_ptr<DistanceAssociate> dis_associate)
{
    std::vector<int> unmatched_all_dets_index = m_associate_out->_unmatched_detections_indices;
    std::vector<int> unmatched_elder_trks_index = m_associate_out->_unmatched_trackers_indices;
    std::vector<int> matched_all_dets_index = m_associate_out->_matched_detections_indices;
    std::vector<int> matched_elder_trks_index = m_associate_out->_matched_trackers_indices;
    xt::xarray<float> dets_for_third = m_associate_out->_unmatched_detections;
    xt::xarray<float> younger_trks_for_third = younger_track_box;

    if (int(dets_for_third.shape(0)) == 0) // 没有待匹配的检测目标
    {
        setout_det(matched_all_dets_index, unmatched_elder_trks_index, matched_elder_trks_index);
    }

    if (int(younger_trks_for_third.shape(0)) == 0)  // 没有待匹配的跟踪目标
    {
        setout_trk(matched_all_dets_index, unmatched_elder_trks_index, matched_elder_trks_index);
    }

    xt::xarray<float> cost_matrix_third;
    if(int(younger_trks_for_third.shape(0)) != 0 && int(dets_for_third.shape(0)) != 0){
        dis_associate->pre_process_associate_data(&dets_for_third, &younger_trks_for_third, 1);
        dis_associate->cal_cost_matrix(cost_matrix_third);
        dis_associate->execute_match(cost_matrix_third, m_associate_out);
        dis_associate->post_process_associate_data_third(m_associate_out, &vaild_elder, &vaild_younger);
    }
}

void Process_lidar::process_lidar_data(xt::xarray<float> *_p_pPcResult)
{
    xt::xarray<float> empty_data = xt::empty<float>({0, 0});
    m_associate_out->_cost_matrix = empty_data;
    m_associate_out->_matched_indices = empty_data;
    m_associate_out->_matched_detections_indices.clear();
    m_associate_out->_matched_trackers_indices.clear();
    m_associate_out->_unmatched_detections_indices.clear();
    m_associate_out->_unmatched_trackers_indices.clear();
    m_associate_out->_unmatched_detections = empty_data;
    m_associate_out->_unmatched_trackers = empty_data;
    m_associate_out->_matched_all_trks_id.clear();

    // 先清除上一帧的 距离计算 结果
    m_cal_dis_result->ano_dis = xt::empty<float>({0, 0});
    m_cal_dis_result->head_dis = xt::empty<float>({0, 0});
    m_cal_dis_result->square_dis = xt::empty<float>({0, 0});
    m_cal_dis_result->ano_dis_thres_matrix = xt::empty<float>({0, 0});
    m_cal_dis_result->head_dis_thres_matrix = xt::empty<float>({0, 0});
    lidar_box = *_p_pPcResult;
}

void Process_lidar::excute_match_lidar_stage_3rd_match(std::vector<FUKalmanBoxTracker> &_trackers, std::shared_ptr<IouAssociate> iou_associate, std::shared_ptr<DistanceAssociate> dis_associate)
{
    track_box = xt::empty<float>({0, 10});
    younger_track_box = xt::empty<float>({0, 10});
    elder_track_box = xt::empty<float>({0, 10});
    vaild_younger.clear();
    vaild_elder.clear();
    int trackers_num = _trackers.size();
    if (trackers_num > 0){
        // x, y, z, w，   l,  h,   theta, speed
        track_box = xt::zeros<float>({trackers_num, 10});
        std::vector<int> vaild_trackers_index;
        std::vector<int> younger_trackers_index;
        std::vector<int> elder_trackers_index;
        for (int i = 0; i < trackers_num; i++)
        {
            track_box(i, 0) = _trackers[i]._kf._x(0, 0); // x
            track_box(i, 1) = _trackers[i]._kf._x(1, 0); // y
            track_box(i, 2) = _trackers[i]._bbox(5);  // z
            track_box(i, 3) = _trackers[i]._bbox(2);  // w
            track_box(i, 4) = _trackers[i]._bbox(3);  // l
            track_box(i, 5) = _trackers[i]._bbox(6);  // h

            float heading_angle;
            if (_trackers[i].last_angle != None && _trackers[i].last_angle != -1000)
            {
                heading_angle = _trackers[i].last_angle;
            }
            else
            {
                heading_angle = _trackers[i]._bbox(4);  // angle
            }
            track_box(i, 6) = heading_angle;      // theta
            track_box(i, 7) = _trackers[i].speed; // speed
            track_box(i, 8) = float(_trackers[i]._type_fusion.fusion_label); // label
            track_box(i, 9) = _trackers[i].hits;

            if (!(xt::any(xt::isnan(_trackers[i]._kf._x))))
                vaild_trackers_index.push_back(i);

            if (_trackers[i].hits < int(m_fusion_parameter["fusion_param"]["young_hits_thresh"]))
                younger_trackers_index.push_back(i);
            else
                elder_trackers_index.push_back(i);
        }
        // 按hits划分跟踪结果
        vaild_younger.reserve(min(vaild_trackers_index.size(), younger_trackers_index.size()));
        vaild_elder.reserve(min(vaild_trackers_index.size(), elder_trackers_index.size()));
        set_intersection(vaild_trackers_index.begin(), vaild_trackers_index.end(), younger_trackers_index.begin(), younger_trackers_index.end(), back_inserter(vaild_younger));
        set_intersection(vaild_trackers_index.begin(), vaild_trackers_index.end(), elder_trackers_index.begin(), elder_trackers_index.end(), back_inserter(vaild_elder));
        younger_track_box = xt::view(track_box, xt::keep(vaild_younger));
        elder_track_box = xt::view(track_box, xt::keep(vaild_elder));
    }

    // 在这里判断检测结果或者轨迹结果是否有数据，如果检测或者没有轨迹数据，不用执行下面的步骤
    process_iou(iou_associate);
    process_second(dis_associate);
    Process_lidar::process_third(dis_associate);
}

void Process_lidar::update_type(std::vector<FUKalmanBoxTracker> &_m_trackers, unsigned long pc_timestamp)
{
    int trackers_num = _m_trackers.size();
    for (int i = 0; i < trackers_num; i++)
    {
        if (std::find(m_associate_out->_unmatched_trackers_indices.begin(), m_associate_out->_unmatched_trackers_indices.end(), i) == m_associate_out->_unmatched_trackers_indices.end())
        { // 表示i不在_unmatched_trackers_indices中,即检测和轨迹匹配成功了，就更新轨迹
            auto detection_indices = xt::where(xt::col(m_associate_out->_matched_indices, 1) > i - 1 && xt::col(m_associate_out->_matched_indices, 1) < i + 1)[0];
            if (int(detection_indices.size()) >0){
                int index = detection_indices[0];
                xt::xarray<float> bbox = xt::view(lidar_box, int(m_associate_out->_matched_indices(index, 0)), xt::all());
                _m_trackers[i].updatewithlidar(bbox, pc_timestamp);
            }
        }
    }
}
