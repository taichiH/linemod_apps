#!/bin/bash

docker run -it --rm \
       --net=host \
       -e DISPLAY=$DISPLAY \
       -e QT_X11_NO_MITSHM=1 \
       -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
       linemod_apps/0.0.0 \
