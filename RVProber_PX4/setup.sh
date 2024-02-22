#!/bin/bash

# declare an array variable
declare -a version=("px4_1_15_0" "px4_1_14_1" "px4_1_14_0" "px4_1_13_3" "px4_1_13_2" "px4_1_13_1" "px4_1_13_0" "px4_1_12_3" "px4_1_12_2" "px4_1_12_1" "px4_1_12_0")

declare -a commit=("3ad2c64" "beb834a" "b8c541d" "1c8ab2a" "46a12a0" "dc7f29e" "6823cbc" "2e8918d" "ba0b512" "1682fd5" "9524e8e")


if [[ ! -v PX4_HOME ]]; then
    echo "PX4_HOME is not set"
elif [[ -z "$PX4_HOME" ]]; then
    echo "PX4_HOME is set to the empty string"
else
    echo "PX4_HOME has the value: $PX4_HOME"
fi

cd $PX4_HOME

# Clone PX4 Git repositories
for i in "${!version[@]}"; do
   echo "------------- $i.Downloading ${version[i]} -------------"

   git clone https://github.com/PX4/PX4-Autopilot.git ${version[i]}
   cd ${version[i]}
   git checkout ${commit[i]}
   git submodule update --init --recursive

   cd $PX4_HOME
   echo "--------------------------------------------------------"
done
