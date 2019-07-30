# App user interface shell script.

# Location of shell script.
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

echo -n "Do you wish to run the robot in simulation (y/n)?\n"
read simulation
if ["$simulation" != "${simulation#[Yy]}"]; then
  export YARP_CLOCK=/clock
fi

export YARP_CLOCK=/clock
cd $DIR/../build/bin
./app_user_interface $simulation
