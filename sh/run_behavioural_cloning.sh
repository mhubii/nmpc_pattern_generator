# Behavioural cloning shell script.

# Location of shell script.
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

export YARP_CLOCK=/clock
cd $DIR/../build/bin
./behavioural_cloning --io_config ../../libs/io_module/configs.yaml --pg_config ../../libs/pattern_generator/configs.yaml --ki_config ../../libs/kinematics/configs.yaml --robot icubGazeboSim --out_loc ../../out
