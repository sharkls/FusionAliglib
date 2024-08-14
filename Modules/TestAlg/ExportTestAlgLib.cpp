#include "ExportTestAlgLib.h"
#include "CTestAlg.h"

extern "C" __attribute__ ((visibility("default"))) ITestAlg* CreateTestAlgObj(const std::string& p_strExePath)
{
    return new CTestAlg(p_strExePath);
}