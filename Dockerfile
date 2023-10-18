FROM dustynv/ros:iron-desktop-l4t-r35.4.1  

WORKDIR /mac_everything 

COPY ./Scripts/mac_commands.sh ./Scripts/mac_commands.sh

RUN echo "source /mac_everything/Scripts/mac_commands.sh" >> /root/.bashrc
RUN echo "source /mac_everything/ws/install/setup.bash" >> /root/.bashrc
RUN apt-get update
RUN apt-get install -y python3-pip && python3 -m pip install --upgrade pip
RUN pip3 install setuptools==58.2.0
RUN apt-get install -y tmux 

CMD ["/bin/bash"]