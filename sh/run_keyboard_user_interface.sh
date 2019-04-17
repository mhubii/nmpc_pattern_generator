# Keyboard user interface shell script.
# Runs with: sh run_keyboard_user_interface command
# Possible commands are:
# uc = user controlled
# bc = behavioural cloning
# ac = autonomous control

# Location of shell script.
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

export YARP_CLOCK=/clock
cd $DIR/../build/bin
./keyboard_user_interface $1
