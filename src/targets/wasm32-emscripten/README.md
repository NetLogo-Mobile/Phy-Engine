```sh
docker build -f src/targets/wasm32-emscripten/Dockerfile -t wasm32-emscripten-pe .
docker run --rm wasm32-emscripten-pe cat /wasm32-emscripten-pe-release.tar.xz > wasm32-emscripten-pe-release.tar.xz
```
