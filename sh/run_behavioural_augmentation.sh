# Behavioural augmentation shell script.

# Location of shell script.
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

# Read the first command line argument.
if ["$1" != "${1#[Yy]}"]; then
  robot="icub"

  cd $DIR/../build/bin
  ./behavioural_augmentation_real_robot --io_config ../../libs/io_module/configs.yaml --pg_config ../../libs/pattern_generator/configs.yaml --ki_config ../../libs/kinematics/configs.yaml --robot $robot --net_loc ../../libs/learning/python/trained_script_module_gd.pt
else
  export YARP_CLOCK=/clock
  robot="icubGazeboSim"

  cd $DIR/../build/bin
  ./behavioural_augmentation_simulation --io_config ../../libs/io_module/configs.yaml --pg_config ../../libs/pattern_generator/configs.yaml --ki_config ../../libs/kinematics/configs.yaml --robot $robot --net_loc ../../libs/learning/python/trained_script_module_rgbd.pt
fi


