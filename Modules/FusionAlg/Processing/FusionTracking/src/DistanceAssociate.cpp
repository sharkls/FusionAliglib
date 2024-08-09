#include "DistanceAssociate.h"
#include "FunctionHub.h"

DistanceAssociate::DistanceAssociate(std::shared_ptr<cal_dis_result>  _cal_dis_result, nlohmann::json &parameter)
{
    m_cal_dis_result = _cal_dis_result.get();
    m_parameter = parameter;
    for (auto item : parameter["fusion_param"]["motor_vehicle_labels"].items())
    {
        motor_vehicle_labels.push_back(int(item.value()));
    }
}

DistanceAssociate::~DistanceAssociate()
{
}

bool DistanceAssociate::adjust_match(int flag, int det_index, int trk_index)
{
    if (flag == 1){
        if (int(m_trackers(trk_index, 8)) == 4){
            if (m_cal_dis_result->head_dis(det_index, trk_index) > float(m_parameter["fusion_param"]["lidar_head_disThreshold_person"])
                || m_cal_dis_result->head_dis(det_index, trk_index) < -float(m_parameter["fusion_param"]["lidar_head_disThreshold_person"])
                || m_cal_dis_result->ano_dis(det_index, trk_index) > float(m_parameter["fusion_param"]["lidar_ano_disThreshold_person"])
                || m_cal_dis_result->square_dis(det_index, trk_index) > FusionFunction::neighbor_search(m_trackers, trk_index, float(m_parameter["fusion_param"]["search_radius_person"]), m_parameter)[quadrant(det_index, trk_index)]){
                return false;
            }
            else{
                return true;
            }
        }
        else{
            if (m_cal_dis_result->head_dis(det_index, trk_index) > (m_cal_dis_result->head_dis_thres_matrix(det_index, trk_index)
                                                                    + float(m_parameter["fusion_param"]["head_dis_thres_forword_adjust"]))
                || m_cal_dis_result->head_dis(det_index, trk_index) < (-m_cal_dis_result->head_dis_thres_matrix(det_index, trk_index)
                                                                       + float(m_parameter["fusion_param"]["head_dis_thres_backword_adjust"]))
                || m_cal_dis_result->ano_dis(det_index, trk_index) > m_cal_dis_result->ano_dis_thres_matrix(det_index, trk_index)
                || (std::find(motor_vehicle_labels.begin(), motor_vehicle_labels.end(), int(m_trackers(trk_index, 8))) != motor_vehicle_labels.end() && m_trackers(trk_index, 7) < 1.5 && m_cal_dis_result->ano_dis(det_index, trk_index) > 2.0)
                || (int(m_trackers(trk_index, 8)) == 1 && m_cal_dis_result->square_dis(det_index, trk_index) > FusionFunction::neighbor_search(m_trackers, trk_index, float(m_parameter["fusion_param"]["search_radius_bicycle"]), m_parameter)[quadrant(det_index, trk_index)])){
                return false;
            }
            else{
                return true;
            }
        }
    }
    else{
        if (m_cal_dis_result->head_dis(det_index, trk_index) > (m_cal_dis_result->head_dis_thres_matrix(det_index, trk_index)
                                                                + float(m_parameter["fusion_param"]["head_dis_thres_forword_adjust"]))
            || m_cal_dis_result->head_dis(det_index, trk_index) < (-m_cal_dis_result->head_dis_thres_matrix(det_index, trk_index)
                                                                   + float(m_parameter["fusion_param"]["head_dis_thres_backword_adjust"]))
            || m_cal_dis_result->ano_dis(det_index, trk_index) > m_cal_dis_result->ano_dis_thres_matrix(det_index, trk_index)){
            return false;
        }
        else{
            return true;
        }
    }
}

void DistanceAssociate::setout_matched(std::vector<int>& matched_det_index, std::vector<int>& matched_trk_index, fusion_match_out *associate_out)
{
    std::vector<int> matched_all_dets_index_first = associate_out->_matched_detections_indices;
    std::vector<int> matched_all_trks_index_first = associate_out->_matched_trackers_indices;
    std::vector<int> matched_all_dets_index_second;
    std::vector<int> matched_all_trks_index_second;
    std::vector<int> matched_all_dets_index_ls;
    std::vector<int> matched_all_trks_index_ls;
    for(auto it = matched_det_index.begin(); it!= matched_det_index.end(); it++){
        matched_all_dets_index_second.push_back(associate_out->_unmatched_detections_indices[*it]);
    }

    for(auto it = matched_trk_index.begin(); it!= matched_trk_index.end(); it++){
        matched_all_trks_index_second.push_back(associate_out->_unmatched_trackers_indices[*it]);
    }

    matched_all_dets_index_ls.insert(matched_all_dets_index_ls.end(), matched_all_dets_index_first.begin(), matched_all_dets_index_first.end());
    matched_all_dets_index_ls.insert(matched_all_dets_index_ls.end(), matched_all_dets_index_second.begin(), matched_all_dets_index_second.end());

    matched_all_trks_index_ls.insert(matched_all_trks_index_ls.end(), matched_all_trks_index_first.begin(), matched_all_trks_index_first.end());
    matched_all_trks_index_ls.insert(matched_all_trks_index_ls.end(), matched_all_trks_index_second.begin(), matched_all_trks_index_second.end());

    if (int(matched_all_dets_index_ls.size()) <= 0)
    {
        associate_out->_matched_indices = xt::empty<int>({0, 2});
    }
    else
    {
        assert(matched_all_dets_index_ls.size() == matched_all_trks_index_ls.size());
        std::vector<std::size_t> shape = {matched_all_dets_index_ls.size(), 1};
        xt::xarray<int> xt_matched_det_index = xt::adapt(matched_all_dets_index_ls, shape);
        xt::xarray<int> xt_matched_trk_index = xt::adapt(matched_all_trks_index_ls, shape);
        associate_out->_matched_indices = xt::concatenate(xt::xtuple(xt_matched_det_index, xt_matched_trk_index), 1);
    }

    associate_out->_matched_detections_indices = matched_all_dets_index_ls;
    associate_out->_matched_trackers_indices = matched_all_trks_index_ls;
}

void DistanceAssociate::setout_unmatched(std::vector<int>& unmatched_det_index, std::vector<int>& unmatched_trk_index, fusion_match_out *associate_out)
{
    std::vector<int> unmatched_all_trks_index_second;
    for(auto it = unmatched_trk_index.begin(); it!= unmatched_trk_index.end(); it++){
        unmatched_all_trks_index_second.push_back(associate_out->_unmatched_trackers_indices[*it]);
    }

    std::vector<int> unmatched_all_dets_index_second;
    for(auto it = unmatched_det_index.begin(); it!= unmatched_det_index.end(); it++){
        unmatched_all_dets_index_second.push_back(associate_out->_unmatched_detections_indices[*it]);
    }

    associate_out->_unmatched_detections_indices = unmatched_all_dets_index_second;
    associate_out->_unmatched_trackers_indices = unmatched_all_trks_index_second;
    if (unmatched_det_index.size()> 0){
        xt::xarray<float> unmatched_det = xt::zeros<float>({int(unmatched_det_index.size()), int(m_detections.shape(1))});
        for(int i = 0; i < unmatched_det_index.size(); i++){

            xt::view(unmatched_det, i, xt::all()) = xt::view(m_detections, unmatched_det_index[i], xt::all());

        }
        associate_out->_unmatched_detections = unmatched_det;
    }
    else{
        associate_out->_unmatched_detections = xt::empty<float>({0, 0});
    }
}

void DistanceAssociate::setout3rd_matched(std::vector<int>& matched_det_index, std::vector<int>& matched_trk_index, fusion_match_out *associate_out)
{
    std::vector<int> matched_all_dets_index = associate_out->_matched_detections_indices;
    std::vector<int> matched_elder_trks_index = associate_out->_matched_trackers_indices;
    std::vector<int> matched_all_trks_index_from_elder;
    std::vector<int> matched_all_trks_index_from_younger;
    std::vector<int> matched_all_dets_index_third;
    std::vector<int> matched_all_dets_index_ls;
    std::vector<int> matched_all_trks_index_ls;
    for(int i = 0; i < int((matched_elder_trks_index.size())); i++){
        matched_all_trks_index_from_elder.push_back((*m_vaild_elder)[matched_elder_trks_index[i]]);
    }
    for(auto it = matched_trk_index.begin(); it!= matched_trk_index.end(); it++){
        matched_all_trks_index_from_younger.push_back((*m_vaild_younger)[*it]);
    }
    for(auto it = matched_det_index.begin(); it!= matched_det_index.end(); it++){
        matched_all_dets_index_third.push_back(associate_out->_unmatched_detections_indices[*it]);
    }
    matched_all_dets_index_ls.insert(matched_all_dets_index_ls.end(), matched_all_dets_index.begin(), matched_all_dets_index.end());
    matched_all_dets_index_ls.insert(matched_all_dets_index_ls.end(), matched_all_dets_index_third.begin(), matched_all_dets_index_third.end());
    matched_all_trks_index_ls.insert(matched_all_trks_index_ls.end(), matched_all_trks_index_from_elder.begin(), matched_all_trks_index_from_elder.end());
    matched_all_trks_index_ls.insert(matched_all_trks_index_ls.end(), matched_all_trks_index_from_younger.begin(), matched_all_trks_index_from_younger.end());

    if (int(matched_all_dets_index_ls.size()) <= 0)
    {
        associate_out->_matched_indices = xt::empty<int>({0, 2});
    }
    else
    {
        assert(matched_all_dets_index_ls.size() == matched_all_trks_index_ls.size());
        std::vector<std::size_t> shape = {matched_all_dets_index_ls.size(), 1};

        xt::xarray<int> xt_matched_det_index = xt::adapt(matched_all_dets_index_ls, shape);
        xt::xarray<int> xt_matched_trk_index = xt::adapt(matched_all_trks_index_ls, shape);

        associate_out->_matched_indices = xt::concatenate(xt::xtuple(xt_matched_det_index, xt_matched_trk_index), 1);
    }
    associate_out->_matched_detections_indices = matched_all_dets_index_ls;
    associate_out->_matched_trackers_indices = matched_all_trks_index_ls;
}

void DistanceAssociate::setout3rd_unmatched(std::vector<int>& unmatched_det_index, std::vector<int>& unmatched_trk_index, fusion_match_out *associate_out)
{
    std::vector<int> unmatched_all_dets_index_third;
    std::vector<int> unmatched_all_trks_index_from_elder;
    std::vector<int> unmatched_all_trks_index_from_younger;
    std::vector<int> unmatched_all_trks_index_ls;
    for(auto it = unmatched_det_index.begin(); it!= unmatched_det_index.end(); it++){
        unmatched_all_dets_index_third.push_back(associate_out->_unmatched_detections_indices[*it]);
    }
    for(int i = 0; i < int((associate_out->_unmatched_trackers_indices.size())); i++){
        unmatched_all_trks_index_from_elder.push_back((*m_vaild_elder)[associate_out->_unmatched_trackers_indices[i]]);
    }
    for(auto it = unmatched_trk_index.begin(); it!= unmatched_trk_index.end(); it++){
        unmatched_all_trks_index_from_younger.push_back((*m_vaild_younger)[*it]);
    }
    unmatched_all_trks_index_ls.insert(unmatched_all_trks_index_ls.end(), unmatched_all_trks_index_from_elder.begin(), unmatched_all_trks_index_from_elder.end());
    unmatched_all_trks_index_ls.insert(unmatched_all_trks_index_ls.end(), unmatched_all_trks_index_from_younger.begin(), unmatched_all_trks_index_from_younger.end());
    associate_out->_unmatched_detections_indices = unmatched_all_dets_index_third;
    associate_out->_unmatched_trackers_indices = unmatched_all_trks_index_ls;

    if (unmatched_det_index.size()> 0){
        xt::xarray<float> unmatched_det = xt::zeros<float>({int(unmatched_det_index.size()), int(m_detections.shape(1))});
        for(int i = 0; i < unmatched_det_index.size(); i++){
            xt::view(unmatched_det, i, xt::all()) = xt::view(m_detections, unmatched_det_index[i], xt::all());
        }
        associate_out->_unmatched_detections = unmatched_det;
    }
    else{
        associate_out->_unmatched_detections = xt::empty<float>({0, 0});
    }
}

void DistanceAssociate::pre_process_associate_data(xt::xarray<float> *_dets, xt::xarray<float> *_trks, int flag)
{
    m_flag = flag;
    m_detections = *_dets;
    m_trackers = *_trks;
}

void DistanceAssociate::cal_cost_matrix(xt::xarray<float> &cost_matrix)
{
    cost_matrix = xt::zeros<float> ({(int)m_detections.shape(0),(int)m_trackers.shape(0)});
    xt::xarray<float> head_dis = xt::zeros<float> ({(int)m_detections.shape(0),(int)m_trackers.shape(0)});
    xt::xarray<float> ano_dis = xt::zeros<float> ({(int)m_detections.shape(0),(int)m_trackers.shape(0)});
    xt::xarray<float> square_dis = xt::zeros<float> ({(int)m_detections.shape(0),(int)m_trackers.shape(0)});
    quadrant = xt::zeros<int> ({(int)m_detections.shape(0),(int)m_trackers.shape(0)});

    FunctionHub::cal_distance(m_detections, m_trackers, head_dis, ano_dis, cost_matrix, quadrant, m_flag);

    m_cal_dis_result->square_dis = cost_matrix;
    m_cal_dis_result->ano_dis = ano_dis;
    m_cal_dis_result->head_dis = head_dis;
}

void DistanceAssociate::execute_match(xt::xarray<float> _cost_matrix, fusion_match_out *associate_out)
{
    xt::xarray<float> cost;
    if (m_flag == 1){  // 雷达的距离匹配
        float head_thres = float(m_parameter["fusion_param"]["lidar_head_disThreshold"]);
        xt::xarray<float> head_dis_thres_matrix = xt::full_like(m_cal_dis_result->head_dis, head_thres);
        float ano_thres = float(m_parameter["fusion_param"]["lidar_ano_disThreshold"]);
        xt::xarray<float> ano_dis_thres_matrix = xt::full_like(m_cal_dis_result->ano_dis, ano_thres);
        // 把计算得到的距离阈值矩阵存储在结构成员变量中
        m_cal_dis_result->head_dis_thres_matrix = head_dis_thres_matrix;
        m_cal_dis_result->ano_dis_thres_matrix = ano_dis_thres_matrix;
        xt::xarray<float> final_dis = abs(m_cal_dis_result->head_dis) + m_cal_dis_result->ano_dis * 3;
        cost = xt::where(final_dis > 60, xt::ones_like(final_dis)*10000, final_dis);
    }

    if (m_flag == 0){   // 相机距离匹配
        cost = m_cal_dis_result->square_dis;
        float head_thres = float(m_parameter["fusion_param"]["camera_head_disThreshold"]);
        xt::xarray<float> head_dis_thres_matrix = xt::full_like(m_cal_dis_result->head_dis, head_thres);
        float ano_thres = float(m_parameter["fusion_param"]["camera_ano_disThreshold"]);
        xt::xarray<float> ano_dis_thres_matrix = xt::full_like(m_cal_dis_result->ano_dis, ano_thres);

        // 把计算得到的距离阈值矩阵存储在结构成员变量中
        m_cal_dis_result->head_dis_thres_matrix = head_dis_thres_matrix;
        m_cal_dis_result->ano_dis_thres_matrix = ano_dis_thres_matrix;
    }

    auto matched_indices = hungarian_alg(cost);
    associate_out->_cost_matrix = cost;  // 计算出来的距离矩阵存储到associate_out中
    associate_out->_matched_indices = matched_indices;
}

void DistanceAssociate::post_process_associate_data(fusion_match_out *associate_out, xt::xarray<float> *_dets)
{
    std::vector<int> matched_det_index;
    std::vector<int> matched_trk_index;
    std::vector<int> unmatched_det_index;
    std::vector<int> unmatched_trk_index;
    for (int i = 0; i < associate_out->_matched_indices.shape(0); i++){
        matched_det_index.push_back(associate_out->_matched_indices(i, 0));
        matched_trk_index.push_back(associate_out->_matched_indices(i, 1));
    }
    for (int i = 0; i < associate_out->_cost_matrix.shape(0); i++){
        if (std::find(matched_det_index.begin(), matched_det_index.end(), i) == matched_det_index.end())
        {
            unmatched_det_index.push_back(i);
        }
    }
    for (int i = 0; i < associate_out->_cost_matrix.shape(1); i++){
        if (std::find(matched_trk_index.begin(), matched_trk_index.end(), i) == matched_trk_index.end())
        {
            unmatched_trk_index.push_back(i);
        }
    }
    matched_det_index.clear();  // 先清空数据
    matched_trk_index.clear();
    for(int i = 0; i < associate_out->_matched_indices.shape(0); i++){
        int det_index = associate_out->_matched_indices(i, 0);
        int trk_index = associate_out->_matched_indices(i, 1);
        bool match_flag = adjust_match(m_flag, det_index, trk_index);
        if(match_flag)
        {
            matched_det_index.push_back(det_index);
            matched_trk_index.push_back(trk_index);
            continue;
        }
        unmatched_det_index.push_back(det_index);
        unmatched_trk_index.push_back(trk_index);
    }
    setout_matched(matched_det_index, matched_trk_index, associate_out);
    setout_unmatched(unmatched_det_index, unmatched_trk_index, associate_out);
}

void DistanceAssociate::post_process_associate_data_third(fusion_match_out *associate_out, std::vector<int> *vaild_elder, std::vector<int> *vaild_younger)
{
    m_vaild_elder = vaild_elder;
    m_vaild_younger = vaild_younger;
    std::vector<int> matched_det_index;
    std::vector<int> matched_trk_index;
    std::vector<int> unmatched_trk_index;
    std::vector<int> unmatched_det_index;
    for (int i = 0; i < associate_out->_matched_indices.shape(0); i++){
        matched_det_index.push_back(associate_out->_matched_indices(i, 0));
        matched_trk_index.push_back(associate_out->_matched_indices(i, 1));
    }
    for (int i = 0; i < associate_out->_cost_matrix.shape(0); i++){
        if (std::find(matched_det_index.begin(), matched_det_index.end(), i) == matched_det_index.end())
        {
            unmatched_det_index.push_back(i);
        }
    }
    for (int i = 0; i < associate_out->_cost_matrix.shape(1); i++){
        if (std::find(matched_trk_index.begin(), matched_trk_index.end(), i) == matched_trk_index.end())
        {
            unmatched_trk_index.push_back(i);
        }
    }
    matched_det_index.clear();  // 先清空数据
    matched_trk_index.clear();
    for(int i = 0; i < associate_out->_matched_indices.shape(0); i++){
        int det_index = associate_out->_matched_indices(i, 0);
        int trk_index = associate_out->_matched_indices(i, 1);
        bool match_flag = adjust_match(1, det_index, trk_index);
        if(match_flag)
        {
            matched_det_index.push_back(det_index);
            matched_trk_index.push_back(trk_index);
            continue;
        }
        unmatched_det_index.push_back(det_index);
        unmatched_trk_index.push_back(trk_index);
    }
    setout3rd_matched(matched_det_index, matched_trk_index, associate_out);
    setout3rd_unmatched(unmatched_det_index, unmatched_trk_index, associate_out);
}
