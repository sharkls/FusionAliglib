#ifndef PARAMGR_H
#define PARAMGR_H

#include "json.hpp"
using json = nlohmann::json;

class ParaMgr
{
public:
    ParaMgr();
    ~ParaMgr();
    bool jsonload(std::string filepath);
    bool get(json &myjson);

private:
    json json_data;
};

#endif //PARAMGR_H