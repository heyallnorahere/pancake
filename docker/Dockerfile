FROM --platform=$BUILDPLATFORM ros:jazzy AS build
SHELL [ "/bin/bash", "-c" ]

WORKDIR /pancake
COPY scripts scripts
COPY docker docker

ARG TARGETOS
ARG TARGETARCH
ARG BUILDARCH

RUN /pancake/scripts/buildenv.sh ${TARGETOS} ${TARGETARCH} ${BUILDARCH}

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

RUN /pancake/scripts/build.sh ${TARGETOS} ${TARGETARCH} ${BUILDARCH}

FROM ros:jazzy AS runtime
SHELL [ "/bin/bash", "-c" ]

WORKDIR /pancake
COPY --from=build /pancake/install /pancake/scripts /pancake/build ./

RUN source /pancake/install/setup.bash && colcon test --ctest-args --output-on-failure --packages-select pancake && colcon test-result --verbose --all

ENTRYPOINT [ "/pancake/scripts/launch.sh", "integrated" ]