# Zoids

Boids, in OpenGL+GLFW using NanoVG. Written in Zig.

Just for trying out the language and tools.

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

![screenshot](https://user-images.githubusercontent.com/26842759/190493039-1252726f-c784-4390-bd3a-1e31fca325af.png)
