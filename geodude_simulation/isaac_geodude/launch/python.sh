#!/bin/bash

OV_PKG_DIR=/isaac-sim

ISAAC_SCRIPT_DIRS=("$OV_PKG_DIR")

# Prepend the path to all arguments passed in
CUR_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
NEW_ARGS=""
for arg in "$@"
do
    NEW_ARGS="${NEW_ARGS} ${CUR_SCRIPT_DIR}/${arg}"
done

pushd ${ISAAC_SCRIPT_DIRS[0]}
./python.sh $NEW_ARGS
popd
