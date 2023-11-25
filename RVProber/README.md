# RVProber

## Parsing valid ranges of configuration parameters
Please refer to <a href="https://github.com/purseclab/PGFUZZ/tree/main/ArduPilot/xml_parse" target="_blank"> this</a>.

## Mapping inputs to states (terms)
Please refer to <a href="https://github.com/purseclab/PGFUZZ/tree/main/ArduPilot/Dynamic%20analysis" target="_blank"> this</a>.

## How to execute it?
### Download ArduPilot
```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git ardupilot_pgfuzz
cd ardupilot_pgfuzz
git checkout ea559a56aa2ce9ede932e22e5ea28eb1df07781c
git submodule update --init --recursive
```

### Check whether ArduPilot works well on your environment
```bash
cd ~/ardupilot_pgfuzz/
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map -w
```

### Add environment variables
You must point to your rvprober and ArduPilot directories.
```bash
export RVPROBER_HOME=/home/hskim/RVProber/
export ARDUPILOT=/home/hskim/ArduPilot/
export ARDUPILOT_HOME=/home/hskim/ArduPilot/arducopter_4_3_0/
export ARDUCOPTER_1=/home/hskim/ArduPilot/arducopter_4_3_0/
export ARDUCOPTER_2=/home/hskim/ArduPilot/arducopter_4_2_0/
export ARDUCOPTER_3=/home/hskim/ArduPilot/arducopter_4_1_0/
export ARDUCOPTER_4=/home/hskim/ArduPilot/arducopter_4_0_0/
```

### Execute RVProber
```commandline
cd ~/pgfuzz/ArduPilot/
python2 rvprober.py -a GPS_spoofing -p -t 50 -m 1 -w
```
```commandline
python2 rvprober.py -a GPS_spoofing -i 2 -t 50 -m 1 -v ArduCopter -w
python2 rvprober.py -a spoofing_gyro -i 2 -t 50 -m 2 -v APMrover2 -w
python2 rvprober.py -a spoofing_opticalflow -i -2 -t 50 -m 4 -v ArduSub -w
python2 rvprober.py -a spoofing_gyro -i 2 -t 120 -m 4 -f 11 -v ArduCopter -e -w
python2 rvprober.py -a spoofing_gyro -i 2 -t 60 -m 5 -s 1 -v APMrover2 -w
```
Details of command options:
```commandline
-a: An attack <GPS_spoofing|GPS_jamming|spoofing_gyro|spoofing_mag|disconnecting_network|spoofing_opticalflow>
-i: Intensity of attack <-10 - 10>
    i) When you select a value among 1 - 10, it multiplies the intensity of the attack.  
    ii) If you select a value among -10 - -1, it runs a hill climbing attack while multiplying the intensity of the attack. 
    Please set at least 20 seconds of the test duration when you choose the hill climbing attack.
-p: Turn on attack profiling <attack_effect|attack_mutate>
-t: Test duration (unit: second)
-m: Probing target 
  1: Probing *types of RVs*
  2: Probing *hardware configurations*
  4: Probing *flight modes*
  5: Probing *software versions*
  6: Probing *configuration parameters*
  7: Probing *environmental conditions*
  10: Probing *all preconditions*
-f: Flight mode <0-27>
    Start to probing flight mode from the given number
    (e.g.,) -f 24 -> probing flight modes from ZIGZAG mode
-v: Choose which vehicle type must be firstly tested <ArduCopter|APMrover2|ArduSub>
-s: Choose from which software version must be tested
-e: Mutate environment (e.g., wind speed and wave speed)
-w: Wipe EEPROM and reload configuration parameters
```

It creates two terminal windows.

### First terminal window
It is a SITL Simulator (Software in the Loop). You can monitor the RV's current states such as altitude and attitude.<br>
<img src="https://github.com/purseclab/PGFUZZ/blob/main/ArduPilot/example/ArduPilot_ex1.jpg">

### Second terminal window
It shows status of fuzzing such as mutated inputs, violated policies, propositional, and global distances.<br>
<img src="https://github.com/purseclab/PGFUZZ/blob/main/ArduPilot/example/ArduPilot_ex2.jpg">


## Evaluate the noise elimination component
Please refer to <a href="https://github.com/purseclab/PGFUZZ/tree/main/ArduPilot/EEN" target="_blank"> this</a>.

