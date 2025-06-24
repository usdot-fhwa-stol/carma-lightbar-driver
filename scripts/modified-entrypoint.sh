#!/bin/bash

#  Copyright (C) 2018-2025 LEIDOS.
#
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

# Intialize the CARMA environment by sourcing the necessary ROS shell scripts
# then run whatever string is passed as argument to this script in that
# initialized context. E.g. "entrypoint.sh roslaunch carma carma_docker.launch"
# runs "roslaunch carma carma_docker.launch" after setting up the environment
# such that ROS and CARMA are on the user's PATH
# Function to handle termination signals

cleanup() {
  echo "SIGINT detected, carma attempting to turn off all lightbar..."
  # Run the shutdown script with a timeout of 5 seconds
  timeout 5s /home/carma/.base-image/shutdown.sh
  # Check if the timeout command succeeded
  if [ $? -eq 124 ]; then
    echo "WARNING: Lightbar shutdown script timed out after 5 seconds. Exiting."
  fi
  exit 0
}

# Register the signal handlers
trap cleanup SIGINT SIGTERM

# Original entrypoint script
if [ $# -eq 0 ]; then
    # If no other command is passed to this script, run bash
    source ~/.base-image/init-env.sh; exec "bash" &
else
    source ~/.base-image/init-env.sh; exec "$@" &
fi

PID=$!

# Wait for the process to exit
wait $PID
