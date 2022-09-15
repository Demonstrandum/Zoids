const std = @import("std");

const glfw_path = "deps/mach-glfw";
const nanovg_path = "deps/nanovg-zig";
// Dependencies.
const glfw = @import("deps/mach-glfw/build.zig");
const nanovg = @import("deps/nanovg-zig/build.zig");
// Packages.
const nanovg_pkg = std.build.Pkg{ .name = "nanovg", .source = std.build.FileSource.relative(nanovg_path ++ "/src/nanovg.zig") };

pub fn build(b: *std.build.Builder) void {
    const target = b.standardTargetOptions(.{});
    const mode = b.standardReleaseOptions();
    // Define executable.
    // TODO: Pick different source-file depending on OS or WASM target.
    const exe = b.addExecutable("zoids", "src/main.zig");
    exe.setTarget(target);
    exe.setBuildMode(mode);

    // Link dependencies.
    if (target.cpu_arch == null or !(target.cpu_arch.? == .wasm32 or target.cpu_arch.? == .wasm64)) {
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
    }
    // Add packages.
    exe.addPackage(glfw.pkg);
    glfw.link(b, exe, .{});
    nanovg.addNanoVGPackage(exe);
    exe.addPackage(std.build.Pkg{
        .name = "nanovg-perf",
        .source = std.build.FileSource.relative(nanovg_path ++ "/examples/perf.zig"),
        .dependencies = &.{nanovg_pkg},
    });
    exe.install();

    // Set up subcommands.
    const run_cmd = exe.run();
    run_cmd.step.dependOn(b.getInstallStep());
    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);

    const exe_tests = b.addTest("src/main.zig");
    exe_tests.setTarget(target);
    exe_tests.setBuildMode(mode);

    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&exe_tests.step);
}
