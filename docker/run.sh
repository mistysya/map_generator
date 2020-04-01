#!/bin/bash
trap : SIGTERM SIGINT

docker run \
        -it \
        --rm \
        --privileged \
        --net=host \
        --ipc=host \
        -v /home/misty/catkin_ws/src/map_generator/:/root/catkin_ws/src/map_generator/ \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -e DISPLAY=$DISPLAY \
        ros:map-generator \
        /bin/bash -c \
        "catkin_make; \
         cp src/map_generator/map1.png devel/lib/map_generator/map1.png; \
         mkdir -p record/images/; \
         ./devel/lib/map_generator/map_generator src/map_generator/src/config.txt; \
         cp -r record/ src/map_generator/record"

