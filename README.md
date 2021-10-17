# 🦾 Gym ROS: ROS wrapper around openAI Gym Library

## 👋 Description

[OpenAI Gym library](https://gym.openai.com/) is a perfect starting point to develop reinforcement learning algorithms. This kind of machine learning algorithms can be very useful when applied to robotics as it allows machines to acomplish tasks in changing environments or learn hard-to-code solutions. [ROS](https://www.ros.org/), in the other hand is the widest used robotics framework in the world. This projects **tries to serve as a bridge between th OpenAI Gym library and the ROS framework**.

## 📜 Table of contents

- [🦾 Gym ROS: ROS wrapper around openAI Gym Library](#-gym-ros-ros-wrapper-around-openai-gym-library)
  - [👋 Description](#-description)
  - [📜 Table of contents](#-table-of-contents)
  - [👷 Development using Gym ROS](#-development-using-gym-ros)
    - [🐋 Using Docker](#-using-docker)

## 👷 Development using Gym ROS

### 🐋 Using Docker

**1. Clone this repo**

```
mkdir -p ~/catkin_make/src && cd ~/catkin_make/src
git clone https://github.com/Matesanz/gym_ros.git
```

**2. Build Dockerfile**

```
cd gym_ros
docker build -f .devcontainer/Dockerfile -t gym_ros .
```

**3. Launch VS Code and run [remote containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) in vscode.**

```
code .
```

Inside vscode do the follwing to install [remote containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)


1. Press `Ctrl+P`
2. Paste `ext install ms-vscode-remote.remote-containers`
3. Press `Enter`

Then Run the docker in development:

1. Press `Ctrl+Shift+P`
2. Type `open folder in container`
3. Press `Enter` *(wait, first time takes some time to run)*


**4. Launch roscore and the Dev environment**

```
roscore
rosrun gym_ros basenode.py
rostopic pub /startSub std_msgs/String "data: '"CartPole-v1"'"  # Change env name according to the one you want to use
```

To visualize Gym environment launch [web_video_server](http://wiki.ros.org/web_video_server) node:

```
rosrun web_video_server web_video_server
```

Now go to your browser and open http://0.0.0.0:8080
