#!/bin/sh

if [ $# -ne 1 ]; then
  echo "Usage: bash build.sh COURSE"
  echo "COURSE = 'Challenge' or 'Advanced'"
  exit 1
fi

mkdir -p output

mkdir -p aichallenge_ws/src
cp -rd ../autoware/aichallenge_ws/src/aichallenge_eval aichallenge_ws/src/
cp -rd ../autoware/aichallenge_ws/src/aichallenge_launch aichallenge_ws/src/
cp -rd ../autoware/aichallenge_ws/src/aichallenge_score_msgs aichallenge_ws/src/

cp -rd ../autoware/nishishinjuku_autoware_map .

docker build -t aichallenge-eval .
