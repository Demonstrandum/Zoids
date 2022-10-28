const std = @import("std");
const builtin = @import("builtin");

const glfw = @import("glfw");
const nano = @import("nanovg");

const c = @cImport({
    @cInclude("glad/glad.h");
});

const zoids = @import("zoids.zig");
const Vec2 = zoids.Vec2;
const Boid = zoids.Boid;
const Boids = zoids.Boids;
const PerfGraph = zoids.PerfGraph;

const Scale = struct {
    fb_width: u32,
    fb_height: u32,
    unscaled_win_width: u32,
    unscaled_win_height: u32,
    win_width: u32,
    win_height: u32,
    /// Window frame size as Vec2.
    frame: Vec2,
    scale_width: f32,
    scale_height: f32,
    aspect: f32,
    /// Pixel ratio for Hi-DPI devices.
    /// How many framebuffer pixels one window pixel correspond to.
    px_ratio: f32,

    const Self = @This();

    pub fn init(win_size: glfw.Window.Size, fb_size: glfw.Window.Size, content_scale: glfw.Window.ContentScale) Self {
        const frame = Vec2{
            .x = @intToFloat(f32, win_size.width) / content_scale.x_scale,
            .y = @intToFloat(f32, win_size.height) / content_scale.y_scale,
        };
        return Self{
            .fb_width = fb_size.width,
            .fb_height = fb_size.height,
            .unscaled_win_width = win_size.width,
            .unscaled_win_height = win_size.height,
            .win_width = @floatToInt(u32, frame.x),
            .win_height = @floatToInt(u32, frame.y),
            .frame = frame,
            .scale_width = content_scale.x_scale,
            .scale_height = content_scale.y_scale,
            .aspect = @intToFloat(f32, win_size.width) / @intToFloat(f32, win_size.height),
            .px_ratio = @intToFloat(f32, fb_size.width) / @intToFloat(f32, win_size.width),
        };
    }

    /// Modifies self to reflect new window size.
    pub fn recompute(self: *Self, win_width: u32, win_height: u32) void {
        const win_size = glfw.Window.Size{ .width = win_width, .height = win_height };
        const fb_size = glfw.Window.Size{ .width = self.fb_width, .height = self.fb_height };
        const content_scale = glfw.Window.ContentScale{ .x_scale = self.scale_width, .y_scale = self.scale_height };
        self.* = Self.init(win_size, fb_size, content_scale);
    }

    pub fn fromWindow(window: glfw.Window) !Self {
        const fb_size = try window.getFramebufferSize();
        const win_size = try window.getSize();
        var scale = glfw.Window.ContentScale{
            .x_scale = 1,
            .y_scale = 1,
        };
        if (!builtin.target.isDarwin()) {
            scale = window.getContentScale() catch scale;
        }
        return Self.init(win_size, fb_size, scale);
    }
};

const Drawer = struct {
    vg: nano,
    window: glfw.Window,
    size: Scale,
    boids: *Boids,
    perf: *PerfGraph,
    font_normal: nano.Font,

    const Self = @This();
    pub fn init(vg: nano, window: glfw.Window, size: Scale, boids: *Boids, perf: *PerfGraph) Self {
        return Self{
            .vg = vg,
            .window = window,
            .size = size,
            .boids = boids,
            .perf = perf,
            .font_normal = undefined,
        };
    }

    /// Load assets.
    pub fn load(self: *Self) void {
        const normal = @embedFile("assets/Inter.ttf");
        self.font_normal = self.vg.createFontMem("sans", normal);
    }

    pub fn resizeCallback(self: *Self, window: glfw.Window, width: u32, height: u32) !void {
        self.window = window;
        self.size.recompute(width, height);
    }

    pub fn refreshCallback(self: *Self, window: glfw.Window) !void {
        self.window = window;
        self.draw();
        try window.swapBuffers();
    }

    pub fn draw(self: *const Self) void {
        // Clear buffer.
        c.glClear(c.GL_COLOR_BUFFER_BIT | c.GL_DEPTH_BUFFER_BIT | c.GL_STENCIL_BUFFER_BIT);
        // Do NanoVG drawing.
        const vg = self.vg;
        vg.beginFrame(@intToFloat(f32, self.size.win_width), @intToFloat(f32, self.size.win_height), self.size.px_ratio);
        vg.resetTransform();
        self.boids.draw(vg);
        self.perf.draw(vg, 5, 5);
        vg.endFrame();
    }
};

// Global drawer needed for callback usage.
// Would just like to use a lambda capturing a pointer to self (Drawer),
// to avoid the global, but I cannot see that this is possible.
var drawer: Drawer = undefined;

fn resizeCallback(window: glfw.Window, width: i32, height: i32) void {
    drawer.resizeCallback(window, @intCast(u32, width), @intCast(u32, height)) catch @panic("resize failed");
}
fn refreshCallback(window: glfw.Window) void {
    drawer.refreshCallback(window) catch @panic("refresh failed");
}

fn loop(alloc: std.mem.Allocator, boids: *Boids) !void {
    // Initialise GLFW.
    try glfw.init(.{});
    defer glfw.terminate();
    const window = try glfw.Window.create(zoids.window_width, zoids.window_height, "Zoids!", null, null, .{
        .context_version_major = 2,
        .context_version_minor = 0,
        .opengl_forward_compat = false,
    });
    defer window.destroy();
    // Load GL via GLAD.
    try glfw.makeContextCurrent(window);
    if (c.gladLoadGL() == 0) return error.GLADInitFailed;
    // Initialise NanoVG.
    var vg = try nano.gl.init(alloc, .{
        .antialias = true,
        .stencil_strokes = false,
        .debug = true,
    });
    defer vg.deinit();
    // Set up OpenGL.
    const size = try Scale.fromWindow(window);
    c.glViewport(0, 0, @intCast(i32, size.fb_width), @intCast(i32, size.fb_height));
    c.glClearColor(0.47, 0.52, 0.60, 1.0);
    // Init for main loop.
    try glfw.swapInterval(if (zoids.enable_vsync) 1 else 0);
    var perf = PerfGraph.init(.fps, "Frame time");
    drawer = Drawer.init(vg, window, size, boids, &perf);
    drawer.load();
    window.setSizeCallback(resizeCallback);
    window.setRefreshCallback(refreshCallback);
    glfw.setTime(0);
    var tick = glfw.getTime();
    // Main loop.
    var frames: u32 = 0;
    while (!window.shouldClose()) : (frames += 1) {
        // Get frame time.
        const tock = glfw.getTime();
        const delta = tock - tick;
        perf.update(@floatCast(f32, delta));
        tick = tock;
        // Update state.
        boids.update(drawer.size.frame);
        // Draw scene.
        drawer.draw();
        // Swap buffers.
        try window.swapBuffers();
        try glfw.pollEvents();
    }
}

pub fn main() anyerror!void {
    std.log.info("Zoids!", .{});
    // Initialise memory arena.
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    // Grab allocator.
    const alloc = arena.allocator();
    // Initialise boids and flock multiarray.
    var boids = try Boids.init(alloc);
    defer boids.deinit(alloc);

    try loop(alloc, &boids);
}
