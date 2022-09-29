# Zoids

Boids, in OpenGL+GLFW using NanoVG. Written in Zig.

Just for trying out the language and tools.

## WASM Web Version

See the boids compiled to WASM online here: [demonstrandum.github.io/Zoids](https://demonstrandum.github.io/Zoids/).

## Dependencies

You must have `glfw` and `nanovg` installed as system packages.

Clone with

```sh
git clone --recurse-submodules -j8 https://github.com/Demonstrandum/Zoids.git
```

## Run

```sh
zig build run
```

## Build WASM

Build `zoids.wasm` and construct a website to `deployment/`.
Must run web-server to serve local files, using Python `http.server`, for example.

```sh
zig build website -Dtarget=wasm32-freestanding
python3 -m http.server 8080 --directory ./deployment/
open http://localhost:8080/
```

---

![screenshot](https://user-images.githubusercontent.com/26842759/193130326-67c4fca0-057e-4643-9b23-f663def9cb73.png)
