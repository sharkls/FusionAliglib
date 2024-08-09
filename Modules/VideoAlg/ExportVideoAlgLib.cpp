#include "ExportVideoAlgLib.h"
#include "CVideoAlg.h"

extern "C" __attribute__ ((visibility("default"))) IVideoAlg* CreateVideoAlgObj(const std::string& p_strExePath)
{
    return new CVideoAlg(p_strExePath);
}