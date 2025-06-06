FROM --platform=$BUILDPLATFORM ros:jazzy AS build
ARG TARGETARCH
ARG BUILDARCH
SHELL [ "/bin/bash", "-c" ]

LABEL org.opencontainers.image.source=https://github.com/heyallnorahere/pancake
LABEL org.opencontainers.image.description="Pancake swerve ^_^"
LABEL org.opencontainers.image.licenses=Apache-2.0

WORKDIR /pancake
COPY vendor vendor
COPY msg msg
COPY srv srv
COPY launch launch
COPY resource resource
COPY package.xml CMakeLists.txt launch.sh ./
COPY include include
COPY src src
COPY test test

RUN source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DSDL_VIDEO=OFF -DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=${TARGETARCH}
RUN if [[ "${TARGETARCH}" == "${BUILDARCH}" ]]; then source /opt/ros/jazzy/setup.bash && colcon test --ctest-args --output-on-failure --packages-select pancake ; colcon test-result --verbose --all ; fi

FROM ros:jazzy AS runtime
WORKDIR /pancake
COPY --from=build /pancake/install /pancake/install
COPY --from=build /pancake/launch.sh /pancake/

ENTRYPOINT [ "/pancake/launch.sh", "integrated" ]