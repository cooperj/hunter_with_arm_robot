FROM lcas.lincoln.ac.uk/lcas/aoc_scenario_base:latest AS base

ENV DEBIAN_FRONTEND=noninteractive
USER root

RUN add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update \
    && apt-get install -qq -y --no-install-recommends \
    git \
    python3-pip \
    python3-rosdep

# get the source tree and analyse it for its package.xml only
FROM base AS sourcefilter
COPY . /tmp/src
# remove everything that isn't package.xml
RUN find /tmp/src -type f \! -name "package.xml" -print | xargs rm -rf

# install all dependencies listed in the package.xml
FROM base AS depbuilder
# copy the reduced source tree (only package.xml) from previous stage
COPY --from=sourcefilter /tmp/src /tmp/src
RUN rosdep update && apt-get update
RUN cd /tmp/src && rosdep install --from-paths . --ignore-src -r -y && cd && rm -rf /tmp/src

FROM depbuilder AS final
# add sudo without password
RUN echo "ALL ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

USER ros

LABEL org.opencontainers.image.target="devcontainer"
# set to domain id 1 as a default
ENV ROS_DOMAIN_ID=1

# default command for a devcontainer
CMD ["/bin/sh", "-c", "while sleep 1000; do :; done"]