
mode=$1 

if [ "$mode" = "manual" ]
then
    #Manual mode requires device mounting for controller (js*)
    #Note: if wanting to do stuff with turtlesim then device event* for 
    #controller must be mounted instead of js*
    echo "Setup for Manual mode."
        #--device=/dev/input/event2 \
        #--device=/dev/input/js0 \
        #--volume /home/macnav/CMPT496/MacEwan-Navigation-Bot/Config_Files/cyclonedds.xml:/tmp/cyclonedds.xml \
    docker run --runtime nvidia -it --rm \
        --network host \
        --volume /home/macnav/CMPT496/MacEwan-Navigation-Bot/ws:/mac_everything/ws \
        --device=/dev/ttyACM0 \
        --name host_mac_bot \
        macnav
else
    echo "Regular setup."
    docker run --runtime nvidia -it --rm \
        --network host \
        --volume /home/macnav/CMPT496/MacEwan-Navigation-Bot/ws:/mac_everything/ws \
        --name host_mac_bot \
        macnav
fi
