const std = @import("std");
const math = std.math;
const rng = std.rand.DefaultPrng;
const builtin = @import("builtin");

const glfw = @import("glfw");
const nano = @import("nanovg");

const PerfGraph = @import("nanovg-perf");

const c = @cImport({
    @cInclude("glad/glad.h");
});

const rad_in_deg = math.pi / 180.0;
const deg_in_rad = 180.0 / math.pi;
// Configuration variables.
const enable_vsync = true;
const boid_count: usize = 120;
const window_width: comptime_int = 1200;
const window_height: comptime_int = 800;
var rnd = rng.init(0);
var rand = rnd.random();

/// Basic 2D vector algebra.
const Vec2 = struct {
    x: f32,
    y: f32,

    fn cartesian(x: f32, y: f32) @This() {
        return @This(){ .x = x, .y = y };
    }

    fn polar(r: f32, theta: f32) @This() {
        return @This(){ .x = r * @cos(theta), .y = r * @sin(theta) };
    }

    fn zero() @This() {
        return @This(){ .x = 0, .y = 0 };
    }

    fn mod(self: @This(), modulus: @This()) @This() {
        return @This(){ .x = @mod(self.x, modulus.x), .y = @mod(self.y, modulus.y) };
    }
    fn add(self: @This(), other: @This()) @This() {
        return @This(){ .x = self.x + other.x, .y = self.y + other.y };
    }
    fn sub(self: @This(), other: @This()) @This() {
        return @This(){ .x = self.x - other.x, .y = self.y - other.y };
    }
    fn scale(self: @This(), scalar: f32) @This() {
        return @This(){ .x = self.x * scalar, .y = self.y * scalar };
    }
    fn normalize(self: @This()) @This() {
        return self.scale(1.0 / self.norm());
    }

    fn dot(self: @This(), other: @This()) f32 {
        return self.x * other.x + self.y * other.y;
    }
    fn quadrature(self: @This()) f32 {
        return self.dot(self);
    }
    fn norm(self: @This()) f32 {
        return math.sqrt(self.quadrature());
    }
    fn angle(self: @This()) f32 {
        return math.atan2(f32, self.y, self.x);
    }

    fn isWithinRadius(self: @This(), r_squared: f32) bool {
        return self.x * self.x + self.y * self.y <= r_squared;
    }
    fn isWithinCircle(self: @This(), centre: @This(), r_squared: f32) bool {
        return self.sub(centre).isWithinCircle(r_squared);
    }
    fn isClockwiseOf(self: @This(), other: @This()) bool {
        return self.x * other.y - self.y * other.x > 0;
    }
    fn isWithinSector(self: @This(), centre: @This(), start_rad: f32, end_rad: f32, r_squared: f32) bool {
        const p = self.sub(centre);
        if (!p.isWithinRadius(r_squared))
            return false;

        const is_clockwise_of_end = p.isClockwiseOf(Vec2.polar(1.0, end_rad));
        const is_clockwise_of_start = p.isClockwiseOf(Vec2.polar(1.0, start_rad));

        if (math.fabs(end_rad - start_rad) < math.pi) {
            return !is_clockwise_of_end and is_clockwise_of_start;
        } else {
            return is_clockwise_of_end or !is_clockwise_of_start;
        }
    }
};

const Boid = struct {
    position: Vec2,
    velocity: Vec2,

    const Self = @This();
    // Boid configuration variables.
    const fov: f32 = 250 * rad_in_deg;
    const sight: f32 = 120;
    const separation_force = 1.45;
    const minimum_velocity = 0.65;
    const fill = nano.rgbaf(0.7, 0.9, 0.8, 0.8);
    const stroke = nano.rgbaf(0.8, 0.9, 1.0, 0.5);
    const focused_color = nano.rgbaf(1.0, 0.8, 0.9, 1.0);
    const observed_color = nano.rgbaf(1.0, 0.9, 0.7, 0.85);

    fn canSee(self: Self, other: Vec2) bool {
        const dir = self.velocity.angle();
        return other.isWithinSector(
            self.position,
            dir - Self.fov / 2,
            dir + Self.fov / 2,
            Self.sight * Self.sight,
        );
    }

    /// Separate boids by computing acceleration though summing
    /// spearation vectors scaled inversly by thier magnitude.
    fn separation(self: Self, neighbours: *Boids.NeighbourIterator) Vec2 {
        var count: f32 = 0;
        var accel = Vec2.zero();
        while (neighbours.next()) |neighbour| {
            const sep = self.position.sub(neighbour.position);
            accel = accel.add(sep.scale(1 / sep.quadrature()));
            count += 1;
        }
        if (count == 0) return accel;
        return accel.scale(separation_force / count);
    }

    fn alignment(self: Self, neighbours: *Boids.NeighbourIterator) Vec2 {
        _ = self;
        _ = neighbours;
        return Vec2.zero();
    }

    fn cohesion(self: Self, neighbours: *Boids.NeighbourIterator) Vec2 {
        _ = self;
        _ = neighbours;
        return Vec2.zero();
    }
};

// FIXME: remove, temporary hack.
var frames: u32 = 0;

/// Manage the flock of boids.
const Boids = struct {
    /// How many neighbours that can be consider at the time.
    const max_neighbours: usize = 32;

    focused_boid: usize = 0,
    flock: std.MultiArrayList(Boid),
    /// Any given boid's neighbours' indices is a slice into this fixed array.
    neighbour_indices: [max_neighbours]usize = [_]usize{0} ** max_neighbours,

    /// Move flock based on their velocities.
    fn move(self: *@This(), bounds: Vec2) void {
        var i: usize = 0;
        for (self.flock.items(.position)) |*pos| {
            const vel = self.flock.get(i).velocity;
            pos.* = pos.add(vel);
            pos.* = pos.mod(bounds);
            i += 1;
        }
    }

    /// Update for one time step.
    pub fn update(self: *@This(), bounds: Vec2) void {
        var i: usize = 0;
        for (self.flock.items(.velocity)) |*velocity| {
            const boid = self.flock.get(i);
            var neighbours = self.neighbourIterator(i);
            const sep = boid.separation(&neighbours);
            neighbours.reset();
            const alg = boid.alignment(&neighbours);
            neighbours.reset();
            const coh = boid.cohesion(&neighbours);
            neighbours.reset();
            const acceleration = sep.add(alg).add(coh);
            velocity.* = velocity.add(acceleration);
            if (velocity.quadrature() < Boid.minimum_velocity * Boid.minimum_velocity)
                velocity.* = velocity.normalize().scale(Boid.minimum_velocity);
            i += 1;
        }
        self.move(bounds);
    }

    /// Compute slice of indices of all neighbours to the boid at the given index.
    fn getNeighbours(self: *@This(), index: usize) []usize {
        const boid = self.flock.get(index);
        const dir = boid.velocity.angle();
        var i: usize = 0;
        var neighbour_count: usize = 0;
        for (self.flock.items(.position)) |pos| {
            if (i != index) {
                const is_seen = pos.isWithinSector(
                    boid.position,
                    dir - Boid.fov / 2,
                    dir + Boid.fov / 2,
                    Boid.sight * Boid.sight,
                );
                if (is_seen) {
                    self.neighbour_indices[neighbour_count] = i;
                    neighbour_count += 1;
                }
            }
            i += 1;
        }

        return self.neighbour_indices[0..neighbour_count];
    }

    const NeighbourIterator = struct {
        count: usize = 0,
        list: []usize,
        flock: *std.MultiArrayList(Boid),

        pub fn next(self: *@This()) ?Boid {
            if (self.count >= self.list.len) return null;
            self.count += 1;
            return self.flock.get(self.list[self.count - 1]);
        }

        pub fn reset(self: *@This()) void {
            self.count = 0;
        }
    };

    fn neighbourIterator(self: *@This(), i: usize) NeighbourIterator {
        return NeighbourIterator{
            .count = 0,
            .list = self.getNeighbours(i),
            .flock = &self.flock,
        };
    }

    fn drawBoid(vg: nano, boid: Boid) void {
        const r = 10;
        const pos = boid.position;
        const vel = boid.velocity;
        // Boid shape.
        vg.save();
        vg.beginPath();
        // Transform coordinates to shape origin.
        vg.translate(pos.x, pos.y);
        vg.rotate(vel.angle());
        vg.translate(-r, -r);
        // Draw boid shape.
        vg.moveTo(0, 0);
        vg.lineTo(2 * r, r);
        vg.lineTo(0, 2 * r);
        vg.lineTo(r, r);
        vg.closePath();
        // Fill in shape colours.
        vg.fill();
        vg.stroke();
        vg.restore();
    }

    /// Draw the boids with NanoVG.
    pub fn draw(self: *@This(), vg: nano) void {
        const focused = self.flock.get(self.focused_boid);
        // Draw a sector for the focused boid's FOV.
        {
            const pos = focused.position;
            const dir = focused.velocity.angle();
            // Sector shape.
            vg.beginPath();
            vg.arc(pos.x, pos.y, Boid.sight, dir - Boid.fov / 2, dir + Boid.fov / 2, .cw);
            vg.lineTo(pos.x, pos.y);
            vg.closePath();
            vg.fillColor(Boid.focused_color);
            vg.globalAlpha(0.3);
            vg.fill();
            // Joining sight-lines.
            vg.strokeColor(Boid.observed_color);
            vg.strokeWidth(2);
            var neighbours = self.neighbourIterator(self.focused_boid);
            while (neighbours.next()) |neighbour| {
                vg.beginPath();
                vg.moveTo(pos.x, pos.y);
                vg.lineTo(neighbour.position.x, neighbour.position.y);
                vg.stroke();
            }
        }

        vg.globalAlpha(1.0);
        vg.fillColor(Boid.fill);
        vg.strokeColor(Boid.stroke);
        vg.lineJoin(.round);
        vg.strokeWidth(1.5);

        var i: usize = 0;
        while (i < self.flock.len) : (i += 1) {
            const boid = self.flock.get(i);
            if (i == self.focused_boid) {
                vg.fillColor(Boid.focused_color);
                drawBoid(vg, boid);
                vg.fillColor(Boid.fill);
            } else if (focused.canSee(boid.position)) {
                vg.fillColor(Boid.observed_color);
                drawBoid(vg, boid);
                vg.fillColor(Boid.fill);
            } else {
                drawBoid(vg, boid);
            }
        }
    }
};

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
    pub fn load(self: *Self, vg: nano) void {
        const normal = @embedFile("assets/Inter.ttf");
        self.font_normal = vg.createFontMem("sans", normal);
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
    const window = try glfw.Window.create(window_width, window_height, "Zoids!", null, null, .{
        .context_version_major = 2,
        .context_version_minor = 1,
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
    try glfw.swapInterval(if (enable_vsync) 1 else 0);
    var perf = PerfGraph.init(.fps, "Frame time");
    drawer = Drawer.init(vg, window, size, boids, &perf);
    drawer.load(vg);
    window.setSizeCallback(resizeCallback);
    window.setRefreshCallback(refreshCallback);
    glfw.setTime(0);
    var tick = glfw.getTime();
    // Main loop.
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
    var boids = Boids{ .focused_boid = 0, .flock = .{} };
    defer boids.flock.deinit(alloc);
    try boids.flock.ensureTotalCapacity(alloc, boid_count);
    // Populate boids multiarray.
    var count: usize = 0;
    while (count < boid_count) : (count += 1) {
        boids.flock.appendAssumeCapacity(.{
            .position = Vec2.cartesian(window_width * rand.float(f32), window_height * rand.float(f32)),
            .velocity = Vec2.polar(1, 2 * math.pi * rand.float(f32)),
        });
    }

    try loop(alloc, &boids);
}
