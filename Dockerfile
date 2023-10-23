FROM dustynv/ros:iron-desktop-l4t-r35.4.1  

WORKDIR /mac_everything 

COPY ./Scripts/mac_commands.sh ./Scripts/mac_commands.sh
COPY ./Packages /mac_everything/Packages
RUN apt-get update

##############################################################################
# Bellow is the setup for X server on the Host

# Add user macbot_user and grant unlimited privileges  
#RUN export uid=1000 gid=1000
#RUN mkdir -p /home/macbot_user
#RUN echo "macbot_user:x:${uid}:${gid}: \
    #macbot_user,,,:/home/macbot_user:/bin/bash" >> /etc/passwd
#RUN echo "macbot_user:x:${uid}:" >> /etc/group
#RUN echo "macbot_user ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/docker_user
#RUN chmod 0440 /etc/sudoers.d/docker_user
#RUN chown ${uid}:${gid} -R /home/macbot_user 
#USER macbot_user
#ENV HOME /home/macbot_user
###############################################################################
RUN apt-get install -y python3-pip 
RUN python3 -m pip install -r ./Packages/requirements.txt

RUN apt-get install -qqy x11-apps xauth 
RUN apt-get install -y tmux 
RUN useradd -m macbot_user && echo "macbot_user:macbot_user" | \ 
    chpasswd && adduser macbot_user sudo
USER macbot_user
COPY ./Config_Files /home/macbot_user
RUN echo "source /mac_everything/Scripts/mac_commands.sh" >> \
    /home/macbot_user/.bashrc
RUN echo "source /mac_everything/ws/install/setup.bash" >> \
    /home/macbot_user/.bashrc
RUN echo "source /opt/ros/iron/install/setup.bash" >> /home/macbot_user/.bashrc 

CMD ["/bin/bash"] 