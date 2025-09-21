const sokol = @import("sokol");
const slog = sokol.log;
const sg = sokol.gfx;
const sapp = sokol.app;
const sglue = sokol.glue;
const vec3 = @import("math.zig").Vec3;
const mat4 = @import("math.zig").Mat4;
const shd = @import("shaders/texcube.glsl.zig");
const std = @import("std");

const state = struct {
    const drone = struct {
        var pos: vec3 = vec3.zero();
        var velo: vec3 = vec3.zero();

        var angVeloX: f32 = 0;
        var rotX: f32 = 0;
    };

    var pass_action: sg.PassAction = .{};
    var pip: sg.Pipeline = .{};
    var bind: sg.Bindings = .{};
    var view: mat4 = mat4.identity();
};

const input_state = struct {
    var upPressed: bool = false;
    var downPressed: bool = false;
    var leftPressed: bool = false;
    var rightPressed: bool = false;
    var pitchUpPressed: bool = false;
    var pitchDownPressed: bool = false;
};

// a vertex struct with position, color and uv-coords
const Vertex = extern struct { x: f32, y: f32, z: f32, color: u32, u: i16, v: i16 };

export fn init() void {
    sg.setup(.{
        .environment = sglue.environment(),
        .logger = .{ .func = slog.func },
    });

    const cs: f32 = 30;

    state.bind.vertex_buffers[0] = sg.makeBuffer(.{
        .data = sg.asRange(&[_]Vertex{
            // zig fmt: off
            .{ .x = -cs, .y = -1.0, .z = -cs, .color = 0xFF0000FF, .u = 0,     .v = 0 },
            .{ .x =  cs, .y = -1.0, .z = -cs, .color = 0xFF0000FF, .u = 32767, .v = 0 },
            .{ .x =  cs, .y =  1.0, .z = -cs, .color = 0xFF0000FF, .u = 32767, .v = 32767 },
            .{ .x = -cs, .y =  1.0, .z = -cs, .color = 0xFF0000FF, .u = 0,     .v = 32767 },

            .{ .x = -cs, .y = -1.0, .z =  cs, .color = 0xFF00FF00, .u = 0,     .v = 0 },
            .{ .x =  cs, .y = -1.0, .z =  cs, .color = 0xFF00FF00, .u = 32767, .v = 0 },
            .{ .x =  cs, .y =  1.0, .z =  cs, .color = 0xFF00FF00, .u = 32767, .v = 32767 },
            .{ .x = -cs, .y =  1.0, .z =  cs, .color = 0xFF00FF00, .u = 0,     .v = 32767 },

            .{ .x = -cs, .y = -1.0, .z = -cs, .color = 0xFFFF0000, .u = 0,     .v = 0 },
            .{ .x = -cs, .y =  1.0, .z = -cs, .color = 0xFFFF0000, .u = 32767, .v = 0 },
            .{ .x = -cs, .y =  1.0, .z =  cs, .color = 0xFFFF0000, .u = 32767, .v = 32767 },
            .{ .x = -cs, .y = -1.0, .z =  cs, .color = 0xFFFF0000, .u = 0,     .v = 32767 },

            .{ .x =  cs, .y = -1.0, .z = -cs, .color = 0xFFFF007F, .u = 0,     .v = 0 },
            .{ .x =  cs, .y =  1.0, .z = -cs, .color = 0xFFFF007F, .u = 32767, .v = 0 },
            .{ .x =  cs, .y =  1.0, .z =  cs, .color = 0xFFFF007F, .u = 32767, .v = 32767 },
            .{ .x =  cs, .y = -1.0, .z =  cs, .color = 0xFFFF007F, .u = 0,     .v = 32767 },

            .{ .x = -cs, .y = -1.0, .z = -cs, .color = 0xFFFF7F00, .u = 0,     .v = 0 },
            .{ .x = -cs, .y = -1.0, .z =  cs, .color = 0xFFFF7F00, .u = 32767, .v = 0 },
            .{ .x =  cs, .y = -1.0, .z =  cs, .color = 0xFFFF7F00, .u = 32767, .v = 32767 },
            .{ .x =  cs, .y = -1.0, .z = -cs, .color = 0xFFFF7F00, .u = 0,     .v = 32767 },

            .{ .x = -cs, .y =  1.0, .z = -cs, .color = 0xFF007FFF, .u = 0,     .v = 0 },
            .{ .x = -cs, .y =  1.0, .z =  cs, .color = 0xFF007FFF, .u = 32767, .v = 0 },
            .{ .x =  cs, .y =  1.0, .z =  cs, .color = 0xFF007FFF, .u = 32767, .v = 32767 },
            .{ .x =  cs, .y =  1.0, .z = -cs, .color = 0xFF007FFF, .u = 0,     .v = 32767 },
            // zig fmt: on
        }),
    });

    // cube index buffer
    state.bind.index_buffer = sg.makeBuffer(.{
        .usage = .{ .index_buffer = true },
        .data = sg.asRange(&[_]u16{
            0,  1,  2,  0,  2,  3,
            6,  5,  4,  7,  6,  4,
            8,  9,  10, 8,  10, 11,
            14, 13, 12, 15, 14, 12,
            16, 17, 18, 16, 18, 19,
            22, 21, 20, 23, 22, 20,
        }),
    });

    // create a small checker-board image and texture view
    state.bind.views[shd.VIEW_tex] = sg.makeView(.{
        .texture = .{
            .image = sg.makeImage(.{
                .width = 4,
                .height = 4,
                .data = init: {
                    var data = sg.ImageData{};
                    data.mip_levels[0] = sg.asRange(&[4 * 4]u32{
                        0xFFFFFFFF, 0xFF000000, 0xFFFFFFFF, 0xFF000000,
                        0xFF000000, 0xFFFFFFFF, 0xFF000000, 0xFFFFFFFF,
                        0xFFFFFFFF, 0xFF000000, 0xFFFFFFFF, 0xFF000000,
                        0xFF000000, 0xFFFFFFFF, 0xFF000000, 0xFFFFFFFF,
                    });
                    break :init data;
                },
            }),
        },
    });

    // ...and a sampler object with default attributes
    state.bind.samplers[shd.SMP_smp] = sg.makeSampler(.{});

    // shader and pipeline object
    state.pip = sg.makePipeline(.{
        .shader = sg.makeShader(shd.texcubeShaderDesc(sg.queryBackend())),
        .layout = init: {
            var l = sg.VertexLayoutState{};
            l.attrs[shd.ATTR_texcube_pos].format = .FLOAT3;
            l.attrs[shd.ATTR_texcube_color0].format = .UBYTE4N;
            l.attrs[shd.ATTR_texcube_texcoord0].format = .SHORT2N;
            break :init l;
        },
        .index_type = .UINT16,
        .depth = .{
            .compare = .LESS_EQUAL,
            .write_enabled = true,
        },
        .cull_mode = .BACK,
    });

    // pass action for clearing the frame buffer
    state.pass_action.colors[0] = .{
        .load_action = .CLEAR,
        .clear_value = .{ .r = 0.25, .g = 0.5, .b = 0.75, .a = 1 },
    };
}

fn rawInputAxis(positive: bool, negative: bool) f32 {
    return if (positive) 1 else (if (negative) -1 else 0);
}

export fn frame() void {
    const dt: f32 = @floatCast(sapp.frameDuration());

    const dUp = vec3.new(state.view.m[0][1], state.view.m[1][1], state.view.m[2][1]);


    const d = &state.drone;

    const yAccel: f32 = rawInputAxis(input_state.upPressed, false);
    const xAccel: f32 = rawInputAxis(input_state.leftPressed, input_state.rightPressed);
    const rotXAccel: f32 = rawInputAxis(input_state.pitchDownPressed, input_state.pitchUpPressed);

    d.velo = vec3.add(d.velo, vec3.mul(dUp, -yAccel));
    d.velo.x += xAccel;

    d.angVeloX += rotXAccel * dt * 100;
    d.rotX += d.angVeloX * dt;

    d.pos = vec3.add(d.pos, vec3.mul(d.velo, dt));
    d.velo.y += 9.81 * dt;

    const veloNorm = vec3.norm(d.velo);
    d.velo = vec3.add(d.velo, vec3.mul(veloNorm, -0.1));

    const groundY: f32 = -1.1;
    
    if(state.drone.pos.y > groundY) {
        state.drone.pos.y = groundY;
        state.drone.velo.y *= -0.4;
        state.drone.velo.x *= 0.5;
    }

    {
        var v = mat4.identity();
        v = mat4.mul(v, mat4.rotate(d.rotX, vec3.right()));
        v = mat4.mul(v, mat4.translate(d.pos));
        state.view = v;
    }

    const model = mat4.identity();

    // projection
    const aspect = sapp.widthf() / sapp.heightf();
    const proj = mat4.persp(90.0, aspect, 0.01, 1000.0);
    
    // vs params
    const vs_params = shd.VsParams{ .mvp = mat4.mul(mat4.mul(proj, state.view), model) };

    // rendering
    sg.beginPass(.{ .action = state.pass_action, .swapchain = sglue.swapchain() });
    sg.applyPipeline(state.pip);
    sg.applyBindings(state.bind);
    sg.applyUniforms(shd.UB_vs_params, sg.asRange(&vs_params));
    sg.draw(0, 36, 1);
    sg.endPass();
    sg.commit();
}

// #INPUT
export fn input(event: ?*const sapp.Event) void {
    const ev = event.?;

    if (ev.type == .KEY_DOWN) {
        switch (ev.key_code) {
            .W => input_state.upPressed = true,
            .S =>  input_state.downPressed = true,
            .A => input_state.leftPressed = true,
            .D => input_state.rightPressed = true,
            .UP => input_state.pitchDownPressed = true,
            .DOWN => input_state.pitchUpPressed = true,
            else => {},
        }
    }

    if (ev.type == .KEY_UP) {
        switch (ev.key_code) {
            .W => input_state.upPressed = false,
            .S =>  input_state.upPressed = false,
            .A => input_state.leftPressed = false,
            .D => input_state.rightPressed = false,
            .UP => input_state.pitchDownPressed = false,
            .DOWN => input_state.pitchUpPressed = false,
            else => {},
        }
    }
}

export fn cleanup() void {
    sg.shutdown();
}

pub fn main() void {
    sapp.run(.{
        .init_cb = init,
        .frame_cb = frame,
        .cleanup_cb = cleanup,
        .event_cb = input,
        .width = 800,
        .height = 600,
        .sample_count = 4,
        .icon = .{ .sokol_default = true },
        .window_title = "texcube.zig",
        .logger = .{ .func = slog.func },
    });
}