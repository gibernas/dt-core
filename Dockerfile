ARG ARCH=arm32v7

FROM gibernas/duckiebot-interface:master19-${ARCH}

RUN ["cross-build-start"]

ARG REPO_PATH="${CATKIN_WS_DIR}/src/dt-ros-core"

# create repo directory
RUN mkdir -p "${REPO_PATH}"

# copy entire repo
COPY . "${REPO_PATH}/"

# build packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin build \
    --workspace ${CATKIN_WS_DIR}/

# turn off ARM emulation
RUN ["cross-build-end"]

LABEL maintainer="Andrea F. Daniele (afdaniele@ttic.edu)"
