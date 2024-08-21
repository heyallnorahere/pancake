FROM ros:iron
SHELL [ "/bin/bash", "-c" ]

WORKDIR /app
COPY vendor vendor
COPY msg msg
COPY launch launch
COPY package.xml CMakeLists.txt launch.sh ./
COPY include include
COPY src src

RUN source /opt/ros/iron/setup.bash && colcon build --cmake-args -DBUILD_CLIENT=OFF
ENTRYPOINT [ "./launch.sh", "robot" ]