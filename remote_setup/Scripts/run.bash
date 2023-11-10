
 #Don't forget to run xhost + to allow container to access display server on host
        #--device=/dev/input/event17 \
        #--device=/dev/ttyUSB0 \
 

 docker run  -it --rm \
        --env DISPLAY=$DISPLAY \
        --network host \
        --name remote_mac_bot \
        --volume /home/dan/MacEwan-Navigation-Bot/remote_setup/ws:/mac_remote/ws \
        --device=/dev/input/js0 \
        remote_macnav