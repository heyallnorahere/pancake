FROM ros:iron
SHELL [ "/bin/bash", "-c" ]

LABEL org.opencontainers.image.source=https://github.com/heyallnorahere/pancake
LABEL org.opencontainers.image.description="Pancake swerve ^_^"
LABEL org.opencontainers.image.licenses=Apache-2.0

WORKDIR /app
COPY vendor vendor
COPY msg msg
COPY launch launch
COPY package.xml CMakeLists.txt launch.sh ./
COPY include include
COPY src src

RUN source /opt/ros/iron/setup.bash && colcon build --cmake-args -DBUILD_CLIENT=OFF
RUN colcon test --ctest-args --output-on-failure --packages-select pancake ; colcon test-result --verbose --all
ENTRYPOINT [ "./launch.sh", "robot" ]