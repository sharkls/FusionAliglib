#include "FusionFunction.h"

std::pair<float, bool> FusionFunction::cal_angle_new(FUKalmanBoxTracker *tracker, nlohmann::json &m_fusion_parameter)
{
    bool state_info_sufficient_flag = true;
    float object_speed = tracker->speed;
    std::vector<xt::xarray<float>> state_list;
    float dis_thresh_min;
    float dis_thresh_max;
    if (object_speed < 0.6)
    {
        if (tracker->state.size() >= 30)
        {
            state_list.assign(tracker->state.end() - 30, tracker->state.end());
            dis_thresh_min = 1.0;
            dis_thresh_max = 5;
        }
        else
        {
            state_list = tracker->state;
            state_info_sufficient_flag = false;
        }
    }
    else if (object_speed < 1.0)
    {
        if (tracker->state.size() >= 25)
        {
            state_list.assign(tracker->state.end() - 25, tracker->state.end());
            dis_thresh_min = object_speed * 2.5 * 0.75;
            dis_thresh_max = object_speed * 2.5 * 5;
        }
        else
        {
            state_list = tracker->state;
            state_info_sufficient_flag = false;
        }
    }
    else if (object_speed < 1.5)
    {
        if (tracker->state.size() >= 20)
        {
            state_list.assign(tracker->state.end() - 20, tracker->state.end());
            dis_thresh_min = object_speed * 2.0 * 0.5;
            dis_thresh_max = object_speed * 2.0 * 5;
        }
        else
        {
            state_list = tracker->state;
            state_info_sufficient_flag = false;
        }
    }
    else if (object_speed < 3)
    {
        if (tracker->state.size() >= 15)
        {
            state_list.assign(tracker->state.end() - 15, tracker->state.end());
            dis_thresh_min = object_speed * 1.5 * 0.5;
            dis_thresh_max = object_speed * 1.5 * 5;
        }
        else
        {
            state_list = tracker->state;
            state_info_sufficient_flag = false;
        }
    }
    else if (object_speed < 5)
    {
        if (tracker->state.size() >= 10)
        {
            state_list.assign(tracker->state.end() - 10, tracker->state.end());
            dis_thresh_min = object_speed * 1.0 * 0.5;
            dis_thresh_max = object_speed * 1.0 * 5;
        }
        else
        {
            state_list = tracker->state;
            state_info_sufficient_flag = false;
        }
    }
    else
    {
        if (tracker->state.size() >= 5)
        {
            state_list.assign(tracker->state.end() - 5, tracker->state.end());
            dis_thresh_min = object_speed * 0.5 * 0.5;
            dis_thresh_max = object_speed * 0.5 * 5;
        }
        else
        {
            state_list = tracker->state;
            state_info_sufficient_flag = false;
        }
    }
    float dis_x, dis_y, dis_len;
    float dis_x_single_frame, dis_y_single_frame, dis_len_single_frame;
    dis_x = state_list[state_list.size() - 1](0) - state_list[0](0);
    dis_y = state_list[state_list.size() - 1](1) - state_list[0](1);
    dis_len = std::sqrt(dis_x * dis_x + dis_y * dis_y);
    if (state_list.size() >= 2)
    {
        dis_x_single_frame = state_list[state_list.size() - 1](0) - state_list[state_list.size() - 2](0);
        dis_y_single_frame = state_list[state_list.size() - 1](1) - state_list[state_list.size() - 2](1);
        dis_len_single_frame = std::sqrt(dis_x_single_frame * dis_x_single_frame + dis_y_single_frame * dis_y_single_frame);
    }
    else
    {
        dis_len_single_frame = dis_len;
    }
    float dis_angle;
    bool track_angle_flag;
    if (state_list.size() >= 2)
    {
        dis_angle = std::atan2(dis_y, dis_x) * 180 / M_PI;
        dis_angle = (-1 * dis_angle - 90);
        dis_angle = std::fmod(dis_angle, 360.0);
        if (dis_angle < 0)
        {
            dis_angle = dis_angle + 360.0;
        }
    }
    else
    {
        dis_angle = tracker->last_angle;
    }
    if (state_info_sufficient_flag && (dis_len > dis_thresh_min && dis_len < dis_thresh_max) && (dis_len_single_frame / dis_len < 0.5))
    {
        track_angle_flag = true;
    }
    else
    {
        track_angle_flag = false;
    }
    return {dis_angle, track_angle_flag};
}

void FusionFunction::smooth_angle(FUKalmanBoxTracker *tracker, std::vector<int> *motor_vehicle_labels)
{
    tracker->angle_flag = 0;
    float ang_thres;
    if (std::find(motor_vehicle_labels->begin(), motor_vehicle_labels->end(), tracker->_type_fusion.fusion_label) != motor_vehicle_labels->end())
    {
        if (tracker->speed < 0.5)
        {
            ang_thres = 0.01;
        }
        else if (tracker->speed >= 0.5 && tracker->speed < 1.2)
        {
            ang_thres = 5;
            if (tracker->direction == -1)
            {
                ang_thres = 0.1;
            }
        }
        else if (tracker->speed >= 1.2 && tracker->speed < 3)
        {
            ang_thres = 4.5;
            if (tracker->direction == -1)
            {
                ang_thres = 0.3;
            }
        }
        else if (tracker->speed >= 3 && tracker->speed < 5)
        {
            ang_thres = 4.5;
            if (tracker->direction == -1)
            {
                ang_thres = 0.3;
            }
        }
        else if (tracker->speed >= 5 && tracker->speed < 7)
        {
            ang_thres = 4;
        }
        else
        {
            ang_thres = 3;
        }
    }
    else
    {
        if (tracker->speed < 0.2)
        {
            ang_thres = 1;
        }
        else
        {
            ang_thres = 5;
        }
    }

    float last_angle = tracker->last_angle;
    float ang_diff = tracker->track_angle - tracker->last_angle;

    if (ang_diff > 180)
    {
        ang_diff = ang_diff - 360;
    }
    else if (ang_diff < -180)
    {
        ang_diff = ang_diff + 360;
    }

    if (abs(ang_diff) >= ang_thres)
    {
        if (ang_diff > 0)
        {
            tracker->track_angle = last_angle + (ang_thres + ang_thres * log10(ang_diff / ang_thres));
            tracker->angle_flag = 1;
        }
        if (ang_diff < 0)
        {
            tracker->track_angle = last_angle - (ang_thres + ang_thres * log10(abs(ang_diff) / ang_thres));
            tracker->angle_flag = 1;
        }
    }

    if (tracker->track_angle > 360)
    {
        tracker->track_angle = std::fmod(tracker->track_angle, 360.0);
    }

    if (tracker->track_angle < 0)
    {
        tracker->track_angle = tracker->track_angle + 360;
    }
}

std::map<int, float> FusionFunction::neighbor_search(xt::xarray<float> &trackers, int idx, float search_radius, nlohmann::json &m_fusion_parameter){
    std::map<int, float> occupy_quadrant;
    occupy_quadrant.insert(pair<int, float>(0, 1000000.0));
    occupy_quadrant.insert(pair<int, float>(1, 1000000.0));
    occupy_quadrant.insert(pair<int, float>(2, 1000000.0));
    occupy_quadrant.insert(pair<int, float>(3, 1000000.0));
    occupy_quadrant.insert(pair<int, float>(4, 1000000.0));
    occupy_quadrant.insert(pair<int, float>(5, 1000000.0));
    occupy_quadrant.insert(pair<int, float>(6, 1000000.0));
    occupy_quadrant.insert(pair<int, float>(7, 1000000.0));
    occupy_quadrant.insert(pair<int, float>(8, 1000000.0));
    if (search_radius == 0.0) return occupy_quadrant;

    for (int i = 0; i < trackers.shape(0); i++){
        if (i == idx) continue;
        if (int(trackers(i, 9)) >= int(m_fusion_parameter["fusion_param"]["sort_min_hits"])){
            float diff_x = trackers(i, 0) - trackers(idx, 0);
            float diff_y = trackers(i, 1) - trackers(idx, 1);
            float dis = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
            if (dis < search_radius){
                int quadrant = 0;
                if (diff_x > 0 && diff_y > 0 && abs(diff_x) < abs(diff_y))
                    quadrant = 1;
                else if (diff_x > 0 && diff_y > 0 && abs(diff_x) > abs(diff_y))
                    quadrant = 2;
                else if (diff_x > 0 && diff_y < 0 && abs(diff_x) > abs(diff_y))
                    quadrant = 3;
                else if (diff_x > 0 && diff_y < 0 && abs(diff_x) < abs(diff_y))
                    quadrant = 4;
                else if (diff_x < 0 && diff_y < 0 && abs(diff_x) < abs(diff_y))
                    quadrant = 5;
                else if (diff_x < 0 && diff_y < 0 && abs(diff_x) > abs(diff_y))
                    quadrant = 6;
                else if (diff_x < 0 && diff_y > 0 && abs(diff_x) > abs(diff_y))
                    quadrant = 7;
                else if (diff_x < 0 && diff_y > 0 && abs(diff_x) < abs(diff_y))
                    quadrant = 8;

                if (quadrant != 0){
                    dis *= 0.3;
                    occupy_quadrant[quadrant] = min(dis, occupy_quadrant[quadrant]);
                }
            }
        }
    }
    return occupy_quadrant;
}

std::pair<float [2], float [2]> FusionFunction::load_chedao(std::unordered_map<int, std::pair<int, float>> &m_chedao){
        nlohmann::json chedao;
        std::string filepath_json = "Configs/Alg/fusion/chedao/chedao.json";
        std::ifstream ifs(filepath_json, std::ios::in);
        ifs >> chedao;
        for (auto item : chedao.items())
        {
            int key = std::atoi(item.key().c_str());
            int lane = std::atoi(std::string(item.value()["lane"][0]).c_str());
            lane = (lane / 1000 * 2 + lane % 1000 / 100 - 1) * 10 + lane % 10;
            float angle = float(item.value()["angle"][0]);
            m_chedao[key] = {lane, angle};
        }

        std::string filepath_csv = "Configs/Alg/fusion/chedao/limit.csv";
        std::ifstream file(filepath_csv, std::ios::in);
        std::string line;
        int count = 0, row = 0, col = 0;
        std::vector<int> data;
        std::pair<float [2], float [2]> limit;
        while (std::getline(file, line, ','))
        {
            row = count / 3;
            col = count % 3;
            if (row == 0)
            {
                if (col == 1 || col == 2)
                    limit.first[col - 1] = std::stof(line);
            }
            else if (row == 1)
            {
                if (col == 0 || col == 1)
                    limit.second[col ] = std::stof(line);
            }
            count++;
        }
        file.close();
        return limit;
    }