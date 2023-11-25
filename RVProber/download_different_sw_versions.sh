#!/bin/bash

#Copter
#verion_name=("arducopter_4_3_0" "arducopter_4_2_0" "arducopter_4_1_0" "arducopter_4_0_0")

#verion_hash=("93448b71380c417644c9082b7b23e80fb982b626" "678921699cb1b2b8141f56cfcfb8333a38e041c2" "ea559a56aa2ce9ede932e22e5ea28eb1df07781c" "49693540bd555a44d30fc33366b1b8a24977e429")

#Rover
#verion_name=("ardurover_4_3_3" "ardurover_4_2_0" "ardurover_4_1_0"
#"ardurover_3_5_2" "ardurover_3_5_0")

#verion_hash=("149fdb20124a6b2cf256dadfd25d69c26ad83d1d" "ce668594cb55708e391ff64f4664bf9ebbe4e2e9" "60a7194b8306ef39bfb6090d8b4427797711ccec"
#"16a1b5fc413610cd7adc73cc12df5b378b0d9e93" "af36fc5ee5a262937e58f1dc850099974bf65bea")

#ArduSub
verion_name=("ardusub_4_1_1" "ardusub_4_1_0" "ardusub_4_0_3" "ardusub_4_0_2"
"ardusub_4_0_1" "ardusub_4_0_0")

verion_hash=("74f8ceeb418d8e0becdcb7a43e0e2b2e2d65b868" "f2af3c7ed2907be914c41d8512654a77498d3870" "96882ed0aeeb822dd80575312b782fe47c352b25" "947bd98caaf6f3a1c86986ecff1f3c5a589f934f"
"a8b440d6f1dc27b67d4a716127d70836b7b827bc" "799ac6f741ea3ac6a2271c1bc749058cd9ca45e4")

printenv ARDUPILOT

for i in ${!verion_name[@]}; do

  path="$ARDUPILOT${verion_name[$i]}"
  index=$((i+1))
  echo "${index}. Downloading ArduPilot v.${verion_name[$i]} into $path"

  git clone https://github.com/ArduPilot/ardupilot.git $path && cd $_
  git checkout ${verion_hash[$i]}
  git submodule update --init --recursive

  if [[ ${verion_name[$i]} == *"copter"* ]]; then
    env_name="ARDUCOPTER_${index}"
  fi

  if [[ ${verion_name[$i]} == *"rover"* ]]; then
    env_name="ARDUROVER_${index}"
  fi

  if [[ ${verion_name[$i]} == *"sub"* ]]; then
    env_name="ARDUSUB_${index}"
  fi

  echo "env_name:${env_name}"

  #export_str="${env_name}=${path}/"
  export_str="export ${env_name}=${path}/"
  echo ${export_str} >> ~/.bashrc
  printenv ${env_name}

  if [[ ${verion_name[$i]} == *"copter"* ]]; then
    python ./Tools/autotest/sim_vehicle.py -v ArduCopter -w &
  fi

  if [[ ${verion_name[$i]} == *"rover"* ]]; then
    python ./Tools/autotest/sim_vehicle.py -v APMrover2 -w &
  fi

  if [[ ${verion_name[$i]} == *"sub"* ]]; then
    python ./Tools/autotest/sim_vehicle.py -v ArduSub -w &
  fi

  sleep 90
  killall sim_vehicle.py

done

# Reload .bashrc settings
source ~/.bashrc