
mode=$1 

if [ "$mode" = "regular" ]
then
    echo "Regular setup."
    docker run --runtime nvidia -it --rm \
        --network host \
        --volume /home/macnav/CMPT496/MacEwan-Navigation-Bot/ws:/mac_everything/ws \
        --name host_mac_bot \
        macnav
else
    echo "Mounted setup"
        #--device=/dev/input/event2 \
        #--device=/dev/input/js0 \
        #--volume /home/macnav/CMPT496/MacEwan-Navigation-Bot/Config_Files/cyclonedds.xml:/tmp/cyclonedds.xml \
    docker run --runtime nvidia -it --rm \
        --network host \
        --volume /home/macnav/CMPT496/MacEwan-Navigation-Bot/ws:/mac_everything/ws \
        --device=/dev/ttyACM0 \
        --name host_mac_bot \
        macnav
fi
