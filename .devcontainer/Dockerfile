# Use ubuntu 20.04
FROM ros:noetic-ros-base-focal

# Install basic tools
RUN apt-get update && apt-get install -y bash-completion git git-core 
RUN apt-get update && apt-get install -y nano curl
RUN apt-get update && apt-get install -y python3-pip

# Install dependency libraries
RUN apt-get update && apt-get install libopencv-dev -y
RUN apt-get update && apt-get install python-opengl xvfb -y
RUN apt-get update && apt-get install -y python3-rospy python3-catkin-tools

# Set bash as default shell
SHELL ["/bin/bash", "-c"]

# Install ROS cv_bridge
RUN mkdir -p ~/cvbridge_build_ws/src && cd ~/cvbridge_build_ws/src
RUN cd ~/cvbridge_build_ws/src && git clone -b noetic https://github.com/ros-perception/vision_opencv.git --verbose && \
    source /opt/ros/noetic/setup.bash && \
    cd ~/cvbridge_build_ws && catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so && \
    catkin config --install && catkin build cv_bridge && source install/setup.bash --extend

# Install ROS web-video-server node
RUN apt-get update && apt-get install -y ros-noetic-web-video-server

# Clean Docker
RUN rm /etc/apt/apt.conf.d/docker-clean

# Create USer
ARG USERNAME=developer
RUN useradd -m $USERNAME
ENV HOME /home/$USERNAME

# # Persist bash history
RUN SNIPPET="export PROMPT_COMMAND='history -a' && export HISTFILE=/commandhistory/.bash_history" \
    && mkdir /commandhistory \
    && touch /commandhistory/.bash_history \
    && chown -R $USERNAME /commandhistory \
    && echo $SNIPPET >> "/home/$USERNAME/.bashrc"

# Work as non-root user
USER $USERNAME
WORKDIR /home/$USERNAME
ENV SHELL /bin/bash

# Enable bash completion
RUN git clone --depth=1 https://github.com/Bash-it/bash-it.git ~/.bash_it && \
    ~/.bash_it/install.sh --silent && \
    rm ~/.bashrc.bak && \
    echo "source /usr/share/bash-completion/bash_completion" >> ~/.bashrc && \
    bash -i -c "bash-it enable completion git"

# Install enhanced console
RUN git clone --depth 1 https://github.com/b4b4r07/enhancd.git ~/.enhancd && \
    echo "source ~/.enhancd/init.sh" >> ~/.bashrc

# Add poetry
RUN curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | python3 -
COPY pyproject.toml .
COPY poetry.lock .
COPY .pre-commit-config.yaml .

# Install python dependencies and add bin to path
RUN source $HOME/.poetry/env && poetry config virtualenvs.create false && poetry install
ENV PATH="${PATH}:/home/developer/.local/bin"

# Add to ROS commands and poetry to PATH
RUN echo 'source $HOME/.poetry/env' >> $HOME/.bashrc
RUN echo 'source /opt/ros/noetic/setup.bash' >> $HOME/.bashrc
