# Configure plugin paths.
echo " "
echo "Configuring Gazebo7 plugin paths."
echo "Previous GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "Script directory: $SCRIPT_DIR"

MY_PLUGIN_PATH=$SCRIPT_DIR
#export GAZEBO_PLUGIN_PATH=

# Start gazebo client.


# Start gazebo with heicub and artificial agent.
#echo "Starting Gazebo7 Server (gzserver)\n"
#gzserver gazebo_heicub.world --verbose
