FROM ros:jazzy
SHELL [ "/bin/bash", "-c" ]

LABEL org.opencontainers.image.source=https://github.com/heyallnorahere/pancake
LABEL org.opencontainers.image.description="Pancake swerve ^_^"
LABEL org.opencontainers.image.licenses=Apache-2.0

WORKDIR /pancake
COPY vendor vendor
COPY msg msg
COPY srv srv
COPY launch launch
COPY package.xml CMakeLists.txt launch.sh ./
COPY include include
COPY src src
COPY test test

RUN source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DSDL_VIDEO=OFF
RUN source /opt/ros/jazzy/setup.bash && colcon test --ctest-args --output-on-failure --packages-select pancake ; colcon test-result --verbose --all
ENTRYPOINT [ "/pancake/launch.sh", "integrated" ]