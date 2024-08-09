#!/bin/sh
CUR_DIR=$(pwd)
echo ${CUR_DIR}
ENV_FILE=~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CUR_DIR/Output/Lib:$CUR_DIR/Submodules/TPL:$CUR_DIR/Submodules/TPL/av_opencv:$CUR_DIR/Submodules/TPL/fastdds:$CUR_DIR/Submodules/TPL/Alg:$CUR_DIR/Submodules/TPL/Alg/libtorch:$CUR_DIR/Submodules/TPL/Alg/TensorRtLib:$CUR_DIR/Submodules/TPL/Alg/TensorRtLib/stubs" >> $ENV_FILE
source $ENV_FILE