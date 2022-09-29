const std = @import("std");
const builtin = @import("builtin");

const wasm = @import("web-wasm");
const gl = @import("web-webgl");
const keys = @import("web-keys");
pub const log = wasm.log;
pub const log_level = .info;
const logger = std.log.scoped(.wasm);

const nano = @import("nanovg");

const zoids = @import("zoids.zig");
const Vec2 = zoids.Vec2;
const Boid = zoids.Boid;
const Boids = zoids.Boids;
const PerfGraph = zoids.PerfGraph;

const Drawer = struct {
    vg: nano,
    width: f32 = zoids.window_width,
    height: f32 = zoids.window_height,
    scale: f32 = 1.0,
    boids: Boids,
    perf: PerfGraph,
    font_normal: nano.Font = undefined,

    const Self = @This();
    pub fn init(vg: nano, boids: Boids, perf: PerfGraph) Self {
        return Self{
            .vg = vg,
            .boids = boids,
            .perf = perf,
        };
    }

    /// Load assets.
    pub fn load(self: *Self) void {
        const normal = @embedFile("assets/Inter.ttf");
        self.font_normal = self.vg.createFontMem("sans", normal);
    }

    pub fn resize(self: *Self, w: f32, h: f32, s: f32) void {
        self.width = w;
        self.height = h;
        self.scale = s;
        gl.glViewport(0, 0, @floatToInt(i32, s * w), @floatToInt(i32, s * h));
    }

    pub fn draw(self: *Self) void {
        // Clear buffer.
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT | gl.GL_STENCIL_BUFFER_BIT);
        // Do NanoVG drawing.
        var vg = self.vg;
        vg.beginFrame(self.width, self.height, self.scale);
        vg.resetTransform();
        self.boids.draw(vg);
        self.perf.draw(vg, 10, 10);
        vg.endFrame();
    }

    pub fn update(self: *Self, tick: f32) f32 {
        const tock = wasm.performanceNow() / 1000.0;
        const delta = tock - tick;
        self.perf.update(delta);
        self.boids.update(.{ .x = self.width, .y = self.height });
        return tock;
    }
};

// Global drawer needed for callback usage.
var drawer: Drawer = undefined;
var time: f32 = undefined;

// Global allocator
var arena: std.heap.ArenaAllocator = undefined;
var gpa: std.heap.GeneralPurposeAllocator(.{ .safety = false }) = undefined;
var global_alloc: std.mem.Allocator = undefined;

export fn onInit() void {
    arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    gpa = .{ .backing_allocator = arena.allocator() };
    global_alloc = gpa.allocator();
    wasm.global_allocator = global_alloc;
    // Initialise NanoVG.
    var vg = nano.gl.init(global_alloc, .{}) catch {
        logger.err("Failed to create NanoVG context.", .{});
        return;
    };
    // Initialise scene,
    var perf = PerfGraph.init(.fps, "Frame time");
    var boids = Boids.init(global_alloc) catch {
        logger.err("Failed to allocate boids.", .{});
        return;
    };
    // Initialise drawer.
    drawer = Drawer.init(vg, boids, perf);
    drawer.load();
    // Initialise time-keeping.
    time = wasm.performanceNow() / 1000.0;
    // Set clear colour.
    gl.glClearColor(0.47, 0.52, 0.60, 1.0);
}

export fn onResize(w: u32, h: u32, s: f32) void {
    drawer.resize(@intToFloat(f32, w), @intToFloat(f32, h), s);
}

export fn onKeyDown(key: u32) void {
    _ = key;
}
export fn onMouseMove(x: i32, y: i32) void {
    _ = x + y;
}

export fn onAnimationFrame() void {
    const tock = drawer.update(time);
    drawer.draw();
    time = tock;
}
