# Keyboard user interface shell script.
# Runs with: sh run_keyboard_user_interface command
# Possible commands are:
# uc = user controlled
# bc = behavioural cloning
# ba = behavioural augmentation

# Location of shell script.
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

echo -n "Do you wish to run the robot in simulation (y/n)?\n"
read simulation
if ["$simulation" != "${simulation#[Yy]}"]; then
  export YARP_CLOCK=/clock
fi

echo -n "In which mode do you want to run? Possible modes are:\n\
         uc = user controlled\n\
         bc = behavioural cloning\n\
         ba = behavioural augmentation\n"
read mode

cd $DIR/../build/bin
./keyboard_user_interface $simulation $mode
