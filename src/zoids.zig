const std = @import("std");
const math = std.math;
const rng = std.rand.DefaultPrng;

const nano = @import("nanovg");

pub const PerfGraph = @import("nanovg-perf");

pub const rad_in_deg = math.pi / 180.0;
pub const deg_in_rad = 180.0 / math.pi;
// Configuration variables.
pub const enable_vsync = true;
pub const boid_count: usize = 220;
pub const window_width: comptime_int = 1200;
pub const window_height: comptime_int = 800;
var rnd = rng.init(0);
var rand = rnd.random();

/// Basic 2D vector algebra.
pub const Vec2 = packed struct {
    x: f32,
    y: f32,

    inline fn fromSimd(other: std.meta.Vector(2, f32)) @This() {
        return @bitCast(@This(), other);
    }
    inline fn toSimd(self: @This()) std.meta.Vector(2, f32) {
        return @bitCast(std.meta.Vector(2, f32), self);
    }
    inline fn cartesian(x: f32, y: f32) @This() {
        return @This(){ .x = x, .y = y };
    }
    inline fn polar(r: f32, theta: f32) @This() {
        return @This(){ .x = r * @cos(theta), .y = r * @sin(theta) };
    }
    inline fn zero() @This() {
        return @This(){ .x = 0, .y = 0 };
    }

    inline fn mod(self: @This(), modulus: @This()) @This() {
        return fromSimd(@mod(self.toSimd(), modulus.toSimd()));
    }
    inline fn add(self: @This(), other: @This()) @This() {
        return fromSimd(self.toSimd() + other.toSimd());
    }
    inline fn sub(self: @This(), other: @This()) @This() {
        return fromSimd(self.toSimd() - other.toSimd());
    }
    inline fn mul(self: @This(), other: @This()) @This() {
        return fromSimd(self.toSimd() * other.toSimd());
    }
    inline fn scale(self: @This(), scalar: f32) @This() {
        return fromSimd(@splat(2, scalar) * self.toSimd());
    }
    inline fn normalize(self: @This()) @This() {
        return self.scale(1.0 / self.norm());
    }

    inline fn dot(self: @This(), other: @This()) f32 {
        return @reduce(.Add, self.toSimd() * other.toSimd());
    }
    inline fn quadrature(self: @This()) f32 {
        return self.dot(self);
    }
    inline fn norm(self: @This()) f32 {
        return math.sqrt(self.quadrature());
    }
    inline fn angle(self: @This()) f32 {
        return math.atan2(f32, self.y, self.x);
    }

    inline fn isWithinRadius(self: @This(), r_squared: f32) bool {
        return self.quadrature() <= r_squared;
    }
    inline fn isWithinCircle(self: @This(), centre: @This(), r_squared: f32) bool {
        return self.sub(centre).isWithinCircle(r_squared);
    }
    inline fn isClockwiseOf(self: @This(), other: @This()) bool {
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

pub const Boid = struct {
    position: Vec2,
    velocity: Vec2,

    const Self = @This();
    // Boid configuration variables.
    const fov: f32 = 250 * rad_in_deg;
    const sight: f32 = 120;
    const separation_force = 1.55;
    const alignment_force = 0.61;
    const cohesion_force = 0.76;
    const force_limit = 0.01;
    const minimum_velocity = 0.65;
    const maximum_velocity = 1.25;
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
            accel = accel.add(sep.scale((sight / 2) / sep.quadrature()));
            count += 1;
        }
        if (count == 0) return accel;
        return accel.scale(separation_force / count);
    }

    fn alignment(self: Self, neighbours: *Boids.NeighbourIterator) Vec2 {
        _ = self;
        var count: f32 = 0;
        var accel = Vec2.zero();
        while (neighbours.next()) |neighbour| {
            accel = accel.add(neighbour.velocity);
            count += 1;
        }
        if (count == 0) return accel;
        return accel.normalize().scale(alignment_force);
    }

    fn cohesion(self: Self, neighbours: *Boids.NeighbourIterator) Vec2 {
        _ = self;
        var count: f32 = 0;
        var centre = Vec2.zero();
        while (neighbours.next()) |neighbour| {
            centre = centre.add(neighbour.position);
            count += 1;
        }
        if (count == 0) return Vec2.zero();
        centre = centre.scale(1 / count);
        const accel = centre.sub(self.position);
        return accel.normalize().scale(cohesion_force);
    }
};

/// Manage the flock of boids.
pub const Boids = struct {
    /// How many neighbours that can be consider at the time.
    const max_neighbours: usize = 32;

    focused_boid: usize = 0,
    flock: std.MultiArrayList(Boid),
    /// Any given boid's neighbours' indices is a slice into this fixed array.
    neighbour_indices: [max_neighbours]usize = undefined,

    /// Initialise boids and flock multiarray.
    pub fn init(alloc: std.mem.Allocator) !@This() {
        var boids = Boids{ .focused_boid = 0, .flock = .{} };
        try boids.flock.ensureTotalCapacity(alloc, boid_count);
        // Populate boids multiarray.
        var count: usize = 0;
        while (count < boid_count) : (count += 1) {
            boids.flock.appendAssumeCapacity(.{
                .position = Vec2.cartesian(window_width * rand.float(f32), window_height * rand.float(f32)),
                .velocity = Vec2.polar(1, 2 * math.pi * rand.float(f32)),
            });
        }
        return boids;
    }

    pub fn deinit(self: *@This(), alloc: std.mem.Allocator) void {
        self.flock.deinit(alloc);
    }

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
            // Do separation.
            const sep = boid.separation(&neighbours);
            neighbours.reset();
            // Do alignment.
            const alg = boid.alignment(&neighbours);
            neighbours.reset();
            // Do cohesion.
            const coh = boid.cohesion(&neighbours);
            neighbours.reset();
            // Sum to resultant acceleration from each three rules
            // then simply add the acceleration vectors to the
            // velocity vector (low effort Euler integration).
            var acceleration = sep.add(alg).add(coh).scale(1.0 / 3.0);
            if (acceleration.quadrature() > Boid.force_limit * Boid.force_limit)
                acceleration = acceleration.normalize().scale(Boid.force_limit);
            velocity.* = velocity.add(acceleration);
            // Sometimes boids can slow down to a halt, so bottom out
            // at a predefined minimum velocity.
            if (velocity.quadrature() < Boid.minimum_velocity * Boid.minimum_velocity)
                velocity.* = velocity.normalize().scale(Boid.minimum_velocity);
            if (velocity.quadrature() > Boid.maximum_velocity * Boid.maximum_velocity)
                velocity.* = velocity.normalize().scale(Boid.maximum_velocity);
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
            defer i += 1;
            if (i == index) continue; // Ignore yourself as neighbour.
            // Simply stop considering neighbours when there are too many.
            if (neighbour_count >= max_neighbours) break;
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

        return self.neighbour_indices[0..neighbour_count];
    }

    /// Normal iterator for neighbour boids, stores a pointer into
    /// the flock array, so can be used from anywhere.
    pub const NeighbourIterator = struct {
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

    /// Generate iterator over boids which are neighbours to the boid at index i.
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
