#########################################################################
#Arguments bellow are used for X server on Host
#--net=host \
#--env DISPLAY=$DISPLAY \
#--volume /tmp/.X11-unix:/tmp/.X11-unix \
#########################################################################

# Prepare target env
CONTAINER_HOME="macbot_user"
HOST_DOCKER_IP=172.17.0.1

echo "" > .Xauthority

#Get the DISPLAY slot
DISPLAY_NUMBER=$(echo $DISPLAY | cut -d. -f1 | cut -d: -f2)

# Extract current authentication cookie
AUTH_COOKIE=$(xauth list ${DISPLAY} | awk '{print $3}')

# Create the new X Authority file
xauth -f .Xauthority add ${HOST_DOCKER_IP}:${DISPLAY_NUMBER} MIT-MAGIC-COOKIE-1\
    ${AUTH_COOKIE}

docker run --runtime nvidia -it --rm \
    --env DISPLAY=${HOST_DOCKER_IP}:${DISPLAY_NUMBER} \
    --volume /home/macnav/CMPT496/MacEwan-Navigation-Bot/ws:/mac_everything/ws \
    --volume ${PWD}/.Xauthority:/home/${CONTAINER_HOME}/.Xauthority \
    --device=/dev/input/event4 \
    --name mac_bot \
    macnav