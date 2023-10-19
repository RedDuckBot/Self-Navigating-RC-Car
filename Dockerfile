FROM dustynv/ros:iron-desktop-l4t-r35.4.1  

WORKDIR /mac_everything 

COPY ./Scripts/mac_commands.sh ./Scripts/mac_commands.sh

RUN apt-get update

# Add user macbot_user and grant unlimited privileges 
RUN export uid=1000 gid=1000
RUN mkdir -p /home/macbot_user
RUN echo "macbot_user:x:${uid}:${gid}:macbot_user,,,:/home/macbot_user:/bin/bash" \
    >> /etc/passwd
RUN echo "macbot_user:x:${uid}:" >> /etc/group
RUN echo "macbot_user ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/docker_user
RUN chmod 0440 /etc/sudoers.d/docker_user
RUN chown ${uid}:${gid} -R /home/macbot_user 
USER macbot_user
ENV HOME /home/macbot_user

RUN echo "source /mac_everything/Scripts/mac_commands.sh" >> /root/.bashrc
RUN echo "source /mac_everything/ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /opt/ros/iron/install/setup.bash" >> /root/.bashrc 
RUN apt-get install -y python3-pip 
RUN pip3 install setuptools==58.2.0
RUN apt-get install -qqy x11-apps 
RUN apt-get install -y tmux 

CMD ["/bin/bash"] 
