#!/bin/bash

cd /output

source /aichallenge/aichallenge_ws/install/setup.bash
ros2 launch aichallenge_launch aichallenge.launch.xml &

# Wait score.js
until [ -f score.json ]
do
  sleep 5
done
