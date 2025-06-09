FROM --platform=$BUILDPLATFORM ros:jazzy AS build
SHELL [ "/bin/bash", "-c" ]

WORKDIR /pancake
COPY scripts scripts

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

ARG TARGETOS
ARG TARGETARCH
ARG BUILDARCH

RUN /pancake/scripts/build.sh ${TARGETOS} ${TARGETARCH} ${BUILDARCH}

FROM ros:jazzy AS runtime
WORKDIR /pancake
COPY --from=build /pancake/install /pancake/scripts ./

ENTRYPOINT [ "/pancake/scripts/launch.sh", "integrated" ]