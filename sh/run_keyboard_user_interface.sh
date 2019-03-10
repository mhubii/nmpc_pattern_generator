# Keyboard user interface shell script.

# Location of shell script.
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

export YARP_CLOCK=/clock
cd $DIR/../build/bin
./keyboard_user_interface
