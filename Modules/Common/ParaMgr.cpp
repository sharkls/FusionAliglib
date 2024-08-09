#include <iostream>
#include <fstream>
#include <unistd.h>
#include <ParaMgr.h>

ParaMgr::ParaMgr()
{
}

ParaMgr::~ParaMgr()
{
}

bool ParaMgr::jsonload(std::string filepath)
{   
    std::cout << "jsonload_path:" << filepath << std::endl;
    std::string::size_type name_pos = filepath.find_last_of('/') + 1;
    std::string filename = filepath.substr(name_pos, filepath.length() - name_pos);
    std::string file_extension = filepath.substr(filepath.find_last_of('.') + 1);
    std::string name = filename.substr(0, filename.rfind("."));
    if (file_extension.compare("json") != 0)
    {
        return false;
    }
    std::ifstream ifs(filepath, std::ios::in);
    if (!ifs.good())
    {
        std::cout << filepath << " No such Json File\n"
                  << std::endl;
        return false;
    }
    if (!(ifs >> (json_data[name])))
    {
        std::cout << filepath << " Error reading Json File\n"
                  << std::endl;
        return false;
    }
    return true;
};

bool ParaMgr::get(json &myjson)
{
    myjson = json_data;
    return true;
};
