FROM emscripten/emsdk:4.0.22

WORKDIR /app

COPY ../../../ .

WORKDIR /app/src/targets/wasm32-emscripten
# TODO: fix permissions
RUN chmod +x build.sh && ./build.sh

WORKDIR /
RUN tar cJf wasm32-emscripten-pe-release.tar.xz /wasm32-emscripten-pe-release
