#! /bin/bash
rm ../data/*
rm ../param/*

# fastddsgen CAllModuleStatus.idl -d ../param -replace
# fastddsgen CAllParam.idl -d ../param -replace
# fastddsgen CTrigAlgParam.idl -d ../param -replace
# fastddsgen CCommandData.idl -d ../param -replace
fastddsgen CCameraParam.idl -d ../param -replace
fastddsgen CLidarParam.idl -d ../param -replace
# fastddsgen CRadarParam.idl -d ../param -replace
# fastddsgen CNetParam.idl -d ../param -replace
# fastddsgen CAuthorizationParam.idl -d ../param -replace
# fastddsgen CStationParam.idl -d ../param -replace
# fastddsgen CStartControlParam.idl -d ../param -replace
# fastddsgen CPcPreprocessParam.idl -d ../param -replace
# fastddsgen CRunMode.idl -d ../param -replace
# fastddsgen CActivityParam.idl -d ../param -replace

fastddsgen CDataBase.idl -d ../data -replace
fastddsgen CPcSrcData.idl -d ../data -replace
fastddsgen CVideoSrcData.idl -d ../data -replace
fastddsgen CAlgResult.idl -d ../data -replace

# fastddsgen CEventB3Data.idl -d ../data -replace
# fastddsgen CEventB4Data.idl -d ../data -replace
# fastddsgen CEventB5Data.idl -d ../data -replace
# fastddsgen CEventData.idl -d ../data -replace
