# #!/bin/bash

# # Absolute path to this script. /home/user/bin/foo.sh
# SCRIPT=$(readlink -f $0)
# # Absolute path this script is in. /home/user/bin
# SCRIPTPATH=`dirname $SCRIPT`
# cd "$SCRIPTPATH"

# # remove the old link
# rm .tmuxinator.yml


# # link the session file to .tmuxinator.yml
# ln session.yml .tmuxinator.yml

# # start tmuxinator
# tmuxinator

#!/bin/bash

usage() {
    echo "  options:"
    echo "      -s: simulated, choices: [true | false]"
    echo "      -g: use GPS, choices: [true | false]"
    echo "      -e: estimator_type, choices: [raw_odometry, mocap_pose]"
    echo "      -r: record rosbag"
    echo "      -t: launch keyboard teleoperation"
    echo "      -n: drone namespace, default is drone0"
}

# Arg parser
while getopts ":sge:rtn:" opt; do
  case ${opt} in
    s )
      simulated="true"
      ;;
    g )
      gps="true"
      ;;
    e )
      estimator_plugin="${OPTARG}"
      ;;
    r )
      record_rosbag="true"
      ;;
    t )
      launch_keyboard_teleop="true"
      ;;
    n )
      drone_namespace="${OPTARG}"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[sgrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

source utils/tools.bash

# Select tmuxinator session file based on architecture
arch=$(uname -m)
if [[ ${arch} == "aarch64" ]]; then
  session_file="utils/session_hw.yml"
else
  session_file="utils/session_sw.yml"
fi

# Shift optional args
shift $((OPTIND -1))

## DEFAULTS
simulated=${simulated:="false"}
gps=${gps:="false"}
estimator_plugin=${estimator_plugin:="raw_odometry"}
record_rosbag=${record_rosbag:="false"}
launch_keyboard_teleop=${launch_keyboard_teleop:="false"}
drone_namespace=${drone_namespace:="drone"}

if [[ ${simulated} == "true" ]]; then
  simulation_config="sim_config/world.json"
  echo simulation_config
fi

# Generate the list of drone namespaces
drone_ns=()
num_drones=1
for ((i=0; i<${num_drones}; i++)); do
  drone_ns+=("$drone_namespace$i")
done

for ns in "${drone_ns[@]}"
do
  if [[ ${ns} == ${drone_ns[0]} ]]; then
    base_launch="true"
  else
    base_launch="false"
  fi 

  tmuxinator start -n ${ns} -p ${session_file} drone_namespace=${ns} gps=${gps} simulation=${simulated} estimator_plugin=${estimator_plugin} &
  wait
done

tmux attach-session -t ${drone_ns[0]}:UI
