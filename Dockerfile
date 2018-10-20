ARG ROS_DISTRO=melodic

FROM ros:$ROS_DISTRO

ARG DOCKER_USER=concaveteam
ARG PROJECT=concaveteam

MAINTAINER Kawin Nikomborirak concavemail@gmail.com

RUN bash -c \
    'useradd -lmG video $DOCKER_USER \
    && mkdir -p /home/$DOCKER_USER/catkin_ws/src/$PROJECT'

COPY . /home/$DOCKER_USER/catkin_ws/src/$PROJECT/

RUN bash -c \
    'apt-get update \
    && apt-get upgrade -y \
    && cd /home/$DOCKER_USER/catkin_ws \
    && rosdep update \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && rosdep install -iy --from-paths src \
    && catkin_make \
    && source /home/$DOCKER_USER/catkin_ws/devel/setup.bash \
    && echo "source ~/catkin_ws/devel/setup.bash" >> /home/$DOCKER_USER/.bashrc \
    && chown -R $DOCKER_USER /home/$DOCKER_USER'

WORKDIR /home/$DOCKER_USER/catkin_ws
USER $DOCKER_USER

WORKDIR /home/$DOCKER_USER
