FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY . /workspace

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
