#FROM dustynv/ros:iron-desktop-l4t-r32.7.1
FROM dustynv/ros:iron-ros-base-l4t-r32.7.1

WORKDIR /mac_everything 

COPY Config_Files/cyclonedds.xml /tmp/cyclonedds.xml
COPY Scripts/mac_commands.sh ./Scripts/mac_commands.sh
COPY Packages /mac_everything/Packages
COPY Config_Files/.tmux.conf /root
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/tmp/cyclonedds.xml
ENV ROS_DOMAIN_ID=0

#RUN apt-get update

#Install packages
RUN apt-get install -y apt-utils
RUN apt-get install -y python3-pip 
RUN apt-get install iputils-ping -y 
RUN python3 -m pip install -r ./Packages/requirements.txt
RUN apt-get install -y tmux 

#Source ros workspaces and custom commands for root user
RUN echo "source /mac_everything/Scripts/mac_commands.sh" >> \
    /root/.bashrc
RUN echo "source /mac_everything/ws/install/setup.bash" >> \
    /root/.bashrc
RUN echo "source /opt/ros/iron/install/setup.bash" >> /root/.bashrc 

CMD ["/bin/bash"] 