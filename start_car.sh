#!/bin/bash

# Check if the number of arguments is 1 (the YAML file)
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <startup_yaml_file_path>"
    exit 1
fi

yaml_file="$1"

# Check if the yq command is available
if ! command -v yq &> /dev/null; then
    echo "Error: 'yq' is not installed. Please install it first with snap install yq"
    exit 1
fi

# Access and print values by keys
IP=$(yq eval '.IP' "$yaml_file")
Username=$(yq eval '.Username' "$yaml_file")

ssh $Username@$IP "cd aimotion_f1tenth_sysem; source install/setup.bash; ros2 launch vehicle_control vehicle_launch.py"  
