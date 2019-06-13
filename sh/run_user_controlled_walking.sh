# User controlled walking shell script.

# Location of shell script.
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

# Read the first command line argument.
if ["$1" != "${1#[Yy]}"]; then
  robot="icub"
  simulation=false
else
  export YARP_CLOCK=/clock
  robot="icubGazeboSim"
  simulation=true
fi

cd $DIR/../build/bin
./user_controlled_walking --io_config ../../libs/io_module/configs.yaml --pg_config ../../libs/pattern_generator/configs.yaml --ki_config ../../libs/kinematics/configs.yaml --robot $robot --simulation $simulation
