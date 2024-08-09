#include "ExportFusionAlgLib.h"
#include "CFusionAlg.h"

extern "C" __attribute__ ((visibility("default"))) IFusionTrackingAlg* CreateFusionTrackingAlgObj(const std::string& p_strExePath)
{
    return new CFusionAlg(p_strExePath);
}