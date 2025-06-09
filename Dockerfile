FROM --platform=$BUILDPLATFORM debian:latest AS build
SHELL [ "/bin/bash", "-c" ]

ARG TARGETOS
ARG TARGETARCH
ARG BUILDARCH

WORKDIR /pancake
COPY scripts scripts

RUN scripts/install_deps.sh ${TARGETARCH} ${BUILDARCH}

LABEL org.opencontainers.image.source=https://github.com/heyallnorahere/pancake
LABEL org.opencontainers.image.description="Pancake swerve ^_^"
LABEL org.opencontainers.image.licenses=Apache-2.0

COPY vendor vendor
COPY msg msg
COPY srv srv
COPY launch launch
COPY resource resource
COPY package.xml CMakeLists.txt ./
COPY include include
COPY src src
COPY test test

RUN source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DSDL_VIDEO=OFF -DCMAKE_SYSTEM_NAME=$(cat scripts/platforms.json | jq -r ".OS.${TARGETOS}") -DCMAKE_SYSTEM_PROCESSOR=$(cat scripts/platforms.json | jq -r ".Architecture.${TARGETARCH}")
RUN if [[ "${TARGETARCH}" == "${BUILDARCH}" ]]; then source /opt/ros/jazzy/setup.bash && colcon test --ctest-args --output-on-failure --packages-select pancake ; colcon test-result --verbose --all ; fi

FROM ros:jazzy AS runtime
WORKDIR /pancake
COPY --from=build /pancake/install /pancake/scripts ./

ENTRYPOINT [ "/pancake/launch.sh", "integrated" ]