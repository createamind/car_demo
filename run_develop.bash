#!/usr/bin/env bash

# Runs a docker container with the image created by build_demo.bash
# Requires
#   docker
#   nvidia-docker 
#   an X server
# Recommended
#   A joystick mounted to /dev/input/js0 or /dev/input/js1

until nvidia-docker ps
do
    echo "Waiting for docker server"
    sleep 1
done

DIR=$(cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

CONTAINER_ID=`nvidia-docker container ls|grep car_demo | awk '{print $1'}`
# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

if [ "${CONTAINER_ID}" == "" ];
then
    nvidia-docker run -it \
      -e DISPLAY \
      -e QT_X11_NO_MITSHM=1 \
      -e XAUTHORITY=$XAUTH \
      -v "$XAUTH:$XAUTH" \
      -v "/tmp/.X11-unix:/tmp/.X11-unix" \
      -v "/etc/localtime:/etc/localtime:ro" \
      -v "/dev/input:/dev/input" \
      -v "$DIR:/home" \
      --privileged \
      --rm=false \
      osrf/car_demo bash /home/.dev_helper/shell.sh
else
    nvidia-docker exec -it \
        -e DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e XAUTHORITY=$XAUTH \
        ${CONTAINER_ID} bash /home/.dev_helper/shell.sh
fi