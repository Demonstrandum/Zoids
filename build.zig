const std = @import("std");

const glfw_path = "deps/mach-glfw";
const nanovg_path = "deps/nanovg-zig";
// Dependencies.
const glfw = @import("deps/mach-glfw/build.zig");
const nanovg = @import("deps/nanovg-zig/build.zig");
// Packages.
const nanovg_pkg = std.build.Pkg{ .name = "nanovg", .source = std.build.FileSource.relative(nanovg_path ++ "/src/nanovg.zig") };

pub fn build(b: *std.build.Builder) !void {
    const target = b.standardTargetOptions(.{});
    const mode = b.standardReleaseOptions();
    const is_wasm_target = target.cpu_arch != null and
        (target.cpu_arch.? == .wasm32 or target.cpu_arch.? == .wasm64);
    // Define executable.
    const exe = if (is_wasm_target)
        b.addSharedLibrary("zoids", "src/wasm.zig", .unversioned)
    else
        b.addExecutable("zoids", "src/glfw.zig");

    exe.setTarget(target);
    exe.setBuildMode(mode);

    // Link dependencies.
    if (is_wasm_target) {
        const web_sources = .{ "wasm", "webgl", "keys" };
        inline for (web_sources) |web| {
            exe.addPackage(std.build.Pkg{
                .name = "web-" ++ web,
                .source = std.build.FileSource.relative(nanovg_path ++ "/examples/web/" ++ web ++ ".zig"),
                .dependencies = &.{},
            });
        }
    } else {
        // Non-WASM arch links against OpenGL+GLFW.
        exe.addIncludePath(nanovg_path ++ "/lib/gl2/include");
        exe.addCSourceFile(nanovg_path ++ "/lib/gl2/src/glad.c", &.{});
        // Link to system OpenGL.
        if (target.isWindows()) { // Windows target.
            exe.linkSystemLibrary("opengl32");
        } else if (target.isDarwin()) { // macOS target.
            exe.linkFramework("OpenGL");
        } else { // GNU+Linux / *BSD target.
            exe.linkSystemLibrary("GL");
        }
        // Add glfw package.
        exe.addPackage(glfw.pkg);
        try glfw.link(b, exe, .{});
    }
    // Add general packages.
    nanovg.addNanoVGPackage(exe);
    exe.addPackage(std.build.Pkg{
        .name = "nanovg-perf",
        .source = std.build.FileSource.relative(nanovg_path ++ "/examples/perf.zig"),
        .dependencies = &.{nanovg_pkg},
    });
    exe.install();

    if (!is_wasm_target) {
        // Set up subcommands.
        const run_cmd = exe.run();
        run_cmd.step.dependOn(b.getInstallStep());
        if (b.args) |args| {
            run_cmd.addArgs(args);
        }

        const run_step = b.step("run", "Run the app");
        run_step.dependOn(&run_cmd.step);
    } else {
        // Build a website from the wasm.
        const wasm_step = b.step("website", "Build wasm website");
        wasm_step.makeFn = buildWasmWebsite;
        wasm_step.dependOn(b.getInstallStep());
    }
}

fn buildWasmWebsite(self: *std.build.Step) !void {
    _ = self;
    const cwd = std.fs.cwd();
    try cwd.makePath("deployment");
    try cwd.copyFile("src/assets/index.html", cwd, "deployment/index.html", .{});
    try cwd.copyFile("zig-out/lib/zoids.wasm", cwd, "deployment/zoids.wasm", .{});
    try cwd.copyFile(nanovg_path ++ "/js/wasm.js", cwd, "deployment/wasm.js", .{});
    try cwd.copyFile(nanovg_path ++ "/js/webgl.js", cwd, "deployment/webgl.js", .{});
    std.log.info("website built to deployment/", .{});
}
