#!/bin/sh

if [ -f "output/score.json" ]; then
  rm output/score.json  
fi

mkdir -p output

docker run \
  -e NVIDIA_VISIBLE_DEVICES=0 -e NVIDIA_DRIVER_CAPABILITIES=utility,graphics,display,compute \
  --env="DISPLAY" \
  -v $PWD/output:/output \
  --net host \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --runtime=nvidia -it --rm -- aichallenge-eval
