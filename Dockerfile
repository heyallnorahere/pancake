FROM --platform=$BUILDPLATFORM ros:jazzy AS build
SHELL [ "/bin/bash", "-c" ]
RUN apt-get update && apt-get install -y jq

LABEL org.opencontainers.image.source=https://github.com/heyallnorahere/pancake
LABEL org.opencontainers.image.description="Pancake swerve ^_^"
LABEL org.opencontainers.image.licenses=Apache-2.0

WORKDIR /pancake
COPY vendor vendor
COPY msg msg
COPY srv srv
COPY launch launch
COPY resource resource
COPY package.xml CMakeLists.txt launch.sh platforms.json ./
COPY include include
COPY src src
COPY test test

ARG TARGETARCH
ARG TARGETOS
ARG BUILDARCH

RUN source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DSDL_VIDEO=OFF -DCMAKE_SYSTEM_NAME=$(cat platforms.json | jq -r ".OS.${TARGETOS}") -DCMAKE_SYSTEM_PROCESSOR=$(cat platforms.json | jq -r ".Architecture.${TARGETARCH}")
RUN if [[ "${TARGETARCH}" == "${BUILDARCH}" ]]; then source /opt/ros/jazzy/setup.bash && colcon test --ctest-args --output-on-failure --packages-select pancake ; colcon test-result --verbose --all ; fi

FROM ros:jazzy AS runtime
WORKDIR /pancake
COPY --from=build /pancake/install /pancake/install
COPY --from=build /pancake/launch.sh /pancake/

ENTRYPOINT [ "/pancake/launch.sh", "integrated" ]