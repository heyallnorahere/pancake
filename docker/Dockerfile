ARG DISTRO=jazzy
FROM --platform=$BUILDPLATFORM ros:${DISTRO} AS build
SHELL [ "/bin/bash", "-c" ]

WORKDIR /pancake
COPY scripts scripts
COPY docker/sources.list.d docker/sources.list.d

ARG TARGETOS
ARG TARGETARCH
ARG BUILDARCH
ARG DISTRO

RUN /pancake/scripts/buildenv.sh ${TARGETOS} ${TARGETARCH} ${BUILDARCH} ${DISTRO}

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

FROM ros:${DISTRO} AS runtime
SHELL [ "/bin/bash", "-c" ]
WORKDIR /pancake

COPY --from=build /pancake/install ./install
COPY --from=build /pancake/scripts ./scripts
COPY --from=build /pancake/build ./build
COPY --from=build /pancake/package.xml .

ARG DISTRO
RUN scripts/test.sh ${DISTRO}

ENTRYPOINT [ "/pancake/scripts/launch.sh", "integrated" ]