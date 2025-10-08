const std = @import("std");

const sokol = @import("sokol");
const slog = sokol.log;
const sg = sokol.gfx;
const sapp = sokol.app;
const sglue = sokol.glue;

const vec3 = @import("math.zig").Vec3;
const mat4 = @import("math.zig").Mat4;

const shd = @import("shaders/texcube.glsl.zig");

const simgui = sokol.imgui;
const ig = @import("cimgui");
const phy = @import("zphysics");
const physics = @import("physics.zig");
const zgltf = @import("zgltf");
const zigimg = @import("zigimg");

const ini = @import("ini.zig");
const gltf = @import("gltf.zig");

const asset = @import("asset.zig");
const AssetBlock = asset.AssetBlock;
const Texture = asset.Texture;
const Mesh = asset.Mesh;
const Primitive = asset.Primitive;
const Node = asset.Node;
const MeshData = asset.MeshData;
const Material = asset.Material;

const c = @cImport({
    @cInclude("Gamepad.h");
});

const max_cubes = 1024;
const max_bodies = 10240;

// MARK: state
const state = struct {
    const drone = struct {
        var pos: vec3 = vec3.zero();
        var velo: vec3 = vec3.zero();
        var angVelo: vec3 = vec3.zero(); // pitch, yaw, roll
        var rot: vec3 = vec3.zero();
    };

    var pass_action: sg.PassAction = .{};
    var pip: sg.Pipeline = .{};
    var bind: sg.Bindings = .{};
    var view: mat4 = mat4.identity();

    var physics_system: *phy.PhysicsSystem = undefined;
    var droneBodyId: phy.BodyId = undefined;

    var cubesBuffer: [max_cubes]WorldCube = undefined;
    var cubes: std.ArrayListUnmanaged(WorldCube) = .{};

    var attachedGamepad: ?*c.struct_Gamepad_device = null; //

    var gamepadInputThrottle: f32 = -1;

    var iterationsToNextGamepadPoll: u8 = 0;
    const iterationsToWaitForGamepadPoll = 60;

    var useGamepad = true;

    var gpa: std.heap.GeneralPurposeAllocator(.{}) = undefined;

    var checkerboard_tex_view: sg.View = undefined;
    var white_tex_view: sg.View = undefined;
};

var configData = struct {
    thrustForceMult: f32 = 200,
    rollPitchTorqueMult: f32 = 0.5,
    yawTorqueMult: f32 = 0.1,
    dragMult: f32 = 2.0,
    angularDragMult: f32 = 0.5,
}{};

const configPath = "config.ini";
const bindingsPath = "bindings.ini";

const Axis = struct {
    id: u8 = 0,
    deadzone: f32 = 0.2,
};

const AxisBindings = struct {
    throttle: Axis = .{ .id = 5 },
    pitch: Axis = .{ .id = 4 },
    roll: Axis = .{ .id = 3 },
    yaw: Axis = .{ .id = 0 },
};

var axisBindings: AxisBindings = .{};

const WorldCube = struct {
    pos: vec3,
    size: vec3,
    bodyId: phy.BodyId,
};

fn createBox(body_interface: *phy.BodyInterface, pos: vec3, size: vec3) void {
    const bodyId = physics.createBoxBody(body_interface, size, pos, false) catch unreachable;
    state.cubes.appendAssumeCapacity(.{ .pos = pos, .size = size, .bodyId = bodyId });
}

fn gamepadOnDeviceAttached(device: [*c]c.struct_Gamepad_device, context: ?*anyopaque) callconv(.c) void {
    _ = context;

    const devicePtr: *c.struct_Gamepad_device = @ptrCast(device.?);

    state.attachedGamepad = devicePtr;

    std.log.info("Controller attached: {s}, buttons: {}, axes {}, vendor/product: {}/{}", .{
        devicePtr.description,
        devicePtr.numAxes,
        devicePtr.numButtons,
        devicePtr.vendorID,
        devicePtr.productID,
    });
}

fn gamepadOnDeviceDetached(device: [*c]c.struct_Gamepad_device, context: ?*anyopaque) callconv(.c) void {
    _ = context;

    const devicePtr: *c.struct_Gamepad_device = @ptrCast(device.?);

    if (devicePtr == state.attachedGamepad)
        state.attachedGamepad = null;
}

fn gamepadOnAxisMove(
    device: [*c]c.struct_Gamepad_device,
    axisId: c_uint,
    value: f32,
    lastValue: f32,
    timestamp: f64,
    context: ?*anyopaque,
) callconv(.c) void {
    _ = device;
    _ = timestamp;
    _ = context;
    // _ = axisId;
    // _ = value;
    _ = lastValue;
    if (axisId == axisBindings.throttle.id)
        state.gamepadInputThrottle = axisInput(value, axisBindings.throttle.deadzone);
    // std.log.info("axis moved: {}, went from: {} to: {}", .{ axisId, lastValue, value });
}

var asset_block: AssetBlock = .{};

// MARK: initSystems()
fn initSystems() !void {

    // Gamepad

    c.Gamepad_deviceAttachFunc(gamepadOnDeviceAttached, null);
    c.Gamepad_deviceRemoveFunc(gamepadOnDeviceDetached, null);
    c.Gamepad_axisMoveFunc(gamepadOnAxisMove, null);
    c.Gamepad_init();

    // Create debug assets

    state.checkerboard_tex_view = sg.makeView(.{
        .texture = .{
            .image = sg.makeImage(.{
                .width = 4,
                .height = 4,
                .data = init: {
                    var data = sg.ImageData{};
                    data.mip_levels[0] = sg.asRange(&[4 * 4]u32{
                        0xFFFFFFFF, 0x00000000, 0xFFFFFFFF, 0x00000000,
                        0x00000000, 0xFFFFFFFF, 0x00000000, 0xFFFFFFFF,
                        0xFFFFFFFF, 0x00000000, 0xFFFFFFFF, 0x00000000,
                        0x00000000, 0xFFFFFFFF, 0x00000000, 0xFFFFFFFF,
                    });
                    break :init data;
                },
            }),
        },
    });

    state.white_tex_view = sg.makeView(.{
        .texture = .{
            .image = sg.makeImage(.{
                .width = 1,
                .height = 1,
                .data = init: {
                    var data = sg.ImageData{};
                    data.mip_levels[0] = sg.asRange(&[_]u32{0xFFFFFFFF});
                    break :init data;
                },
            }),
        },
    });

    // Physics #INITPHYSICS

    const alloc = std.heap.page_allocator;

    try phy.init(alloc, .{});

    const broadphase_layer_interface = try alloc.create(physics.BroadPhaseLayerInterface);
    broadphase_layer_interface.* = physics.BroadPhaseLayerInterface.init();

    const object_vs_broad_phase_layer_filter = try alloc.create(physics.ObjectVsBroadPhaseLayerFilter);
    object_vs_broad_phase_layer_filter.* = .{};

    const object_layer_pair_filter = try alloc.create(physics.ObjectLayerPairFilter);
    object_layer_pair_filter.* = .{};

    const contact_listener = try alloc.create(physics.ContactListener);
    contact_listener.* = .{};

    state.physics_system = try phy.PhysicsSystem.create(
        @as(*const phy.BroadPhaseLayerInterface, @ptrCast(broadphase_layer_interface)),
        @as(*const phy.ObjectVsBroadPhaseLayerFilter, @ptrCast(object_vs_broad_phase_layer_filter)),
        @as(*const phy.ObjectLayerPairFilter, @ptrCast(object_layer_pair_filter)),
        .{
            .max_bodies = max_bodies,
            .num_body_mutexes = 0,
            .max_body_pairs = 1024,
            .max_contact_constraints = 1024,
        },
    );

    defer state.physics_system.optimizeBroadPhase();

    const body_interface = state.physics_system.getBodyInterfaceMut();

    // physics spawning

    // #DRONEINIT
    state.droneBodyId = try physics.createBoxBody(body_interface, vec3.new(0.1, 0.1, 0.1), vec3.new(0, 1, 20), true);

    state.cubes = std.ArrayListUnmanaged(WorldCube).initBuffer(&state.cubesBuffer);

    // Ground
    createBox(body_interface, vec3.new(0, -100, 0), vec3.new(1000, 100, 1000));

    // Load map from gltf

    {
        state.gpa = .init;

        const gltf_buffer: []align(4) const u8 = try std.fs.cwd().readFileAllocOptions(
            state.gpa.allocator(),
            "art/map.glb",
            std.math.maxInt(usize),
            null,
            .@"4",
            null,
        );
        defer state.gpa.allocator().free(gltf_buffer);

        asset_block = try gltf.load(state.gpa.allocator(), gltf_buffer);
    }

    // Create mesh colliders for map #COLLIDERS

    {
        std.log.info("-- Creating colliders --", .{});

        for (asset_block.meshes.items, 0..) |mesh, mi| {
            for (mesh.primitives.items, 0..) |*primitive, pi| {
                if (primitive.data) |mesh_data| {
                    std.log.info("- mesh: {} prim: {}", .{ mi, pi });
                    primitive.collider = try physics.createMeshCollider(mesh_data);
                }
            }
        }

        std.log.info("-- Creating physics for nodes --", .{});

        var total_bodies: i32 = 0;
        for (asset_block.nodes.items) |*node| {
            std.log.info("Making physics body for node: {s}", .{node.name});
            if (node.mesh) |mesh| {
                for (mesh.primitives.items) |primitive| {
                    std.log.info("- prim: {}", .{total_bodies});
                    if (primitive.collider) |collider| {
                        _ = try physics.addStaticBody(
                            body_interface,
                            collider,
                            node.m.getTranslation(),
                            node.m.getRotation(),
                        );
                    }
                    total_bodies += 1;
                }
            }
        }
    }
}

// #INIT MARK: init()
export fn init() void {
    sg.setup(.{
        .environment = sglue.environment(),
        .logger = .{ .func = slog.func },
        .buffer_pool_size = 1024,
    });

    simgui.setup(.{
        .logger = .{ .func = slog.func },
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
            l.attrs[shd.ATTR_texcube_texcoord0].format = .FLOAT2;
            break :init l;
        },
        .index_type = .UINT32,
        .depth = .{
            .compare = .LESS_EQUAL,
            .write_enabled = true,
        },
        .cull_mode = .BACK,
        .face_winding = .CCW,
    });

    // pass action for clearing the frame buffer
    state.pass_action.colors[0] = .{
        .load_action = .CLEAR,
        .clear_value = .{ .r = 0.25, .g = 0.5, .b = 0.75, .a = 1 },
    };

    initSystems() catch unreachable;
}

fn keyAxisInput(negative: bool, positive: bool) f32 {
    return if (negative) -1 else (if (positive) 1 else 0);
}

fn axisInput(rawInput: f32, deadzone: f32) f32 {
    const abs = @max(0, (@abs(rawInput) * (1 + deadzone * 2) - deadzone * 2));
    if (std.math.sign(rawInput) > 0)
        return abs
    else
        return -abs;
}

fn drawCube(vp: *const mat4, pos: vec3, size: vec3) void {
    const scale = mat4.scale(size);

    const model = mat4.translate(pos).mul(scale);

    const vs_params = shd.VsParams{ .mvp = vp.mul(model) };

    sg.applyUniforms(shd.UB_vs_params, sg.asRange(&vs_params));
    sg.draw(0, 36, 1);
}

// #LOOP MARK: frame()
export fn frame() void {
    simgui.newFrame(.{
        .width = sapp.width(),
        .height = sapp.height(),
        .delta_time = sapp.frameDuration(),
        .dpi_scale = 1,
    });

    const dt: f32 = @floatCast(sapp.frameDuration());

    // Move to mat4
    const dUp = vec3.new(state.view.m[0][1], state.view.m[1][1], state.view.m[2][1]);
    const dRight = vec3.new(state.view.m[0][0], state.view.m[1][0], state.view.m[2][0]);
    //const dForward = vec3.new(state.view.m[0][2], state.view.m[1][2], state.view.m[2][2]);

    //dUp = dUp.add(dForward.mul(-0.2)).norm();

    // #input

    if (state.iterationsToNextGamepadPoll > state.iterationsToWaitForGamepadPoll) {
        state.iterationsToNextGamepadPoll = 0;
        c.Gamepad_detectDevices();
    }
    state.iterationsToNextGamepadPoll += 1;

    c.Gamepad_processEvents();

    var yAccel: f32 = keyAxisInput(false, input_state.throttle_up);
    var pitchAccel: f32 = keyAxisInput(input_state.pitch_down, input_state.pitch_up);
    var rollAccel: f32 = keyAxisInput(input_state.roll_left, input_state.roll_right);
    var yawAccel: f32 = keyAxisInput(input_state.yaw_left, input_state.yaw_right);

    if (state.useGamepad) {
        if (state.attachedGamepad) |gpad| {
            yAccel = (1 + state.gamepadInputThrottle) * 0.5; // gpad.axisStates[5]
            yawAccel = axisInput(gpad.axisStates[axisBindings.yaw.id], axisBindings.yaw.deadzone);
            rollAccel = axisInput(gpad.axisStates[axisBindings.roll.id], axisBindings.roll.deadzone);
            pitchAccel = axisInput(gpad.axisStates[axisBindings.pitch.id], axisBindings.pitch.deadzone);
        }
    }

    yAccel = std.math.clamp(yAccel, 0, 1);
    pitchAccel = std.math.clamp(pitchAccel, -1, 1);
    rollAccel = std.math.clamp(rollAccel, -1, 1);
    yawAccel = std.math.clamp(yawAccel, -1, 1);

    // physics #PHYSICSUPDATE

    const mutBodies = state.physics_system.getBodiesMutUnsafe();

    for (mutBodies) |body| {
        if (!phy.isValidBodyPointer(body) or body.motion_properties == null) continue;

        if (body.id == state.droneBodyId) {
            // #DRONEUPDATE
            const upForce = vec3.mul(dUp, yAccel * configData.thrustForceMult);
            body.addForce(upForce.asArr());
            body.addTorque(.{
                -configData.rollPitchTorqueMult * pitchAccel,
                configData.yawTorqueMult * yawAccel,
                configData.rollPitchTorqueMult * rollAccel,
            });

            body.applyBuoyancyImpulse(
                .{ 0, body.position[1] + 100, 0 },
                .{ 0, 1, 0 },
                0.01,
                configData.dragMult,
                configData.angularDragMult,
                .{ 0, 0, 0 },
                .{ 0, -9.81, 0 },
                dt,
            );
        }
    }

    const bodies = state.physics_system.getBodiesUnsafe();

    var speedKmH: f32 = 0;

    for (bodies) |body| {
        if (!phy.isValidBodyPointer(body) or body.motion_properties == null) continue;

        if (body.motion_type == .dynamic) {
            var v = mat4.identity();
            const dpos = body.getWorldTransform().position;

            const r = body.getWorldTransform().rotation;

            speedKmH = vec3.fromArr(body.getLinearVelocity()).len() * 3.6;

            const camTilt = mat4.rotate(30, dRight);
            const rotMat = mat4.rotateFromMat3(&r);

            v = v.mul(rotMat);
            v = v.mul(camTilt);
            v = v.mul(mat4.translate(vec3.new(dpos[0], dpos[1], dpos[2]).mul(-1)));

            state.view = v;
        }
    }

    state.physics_system.update(dt, .{ .collision_steps = 10 }) catch unreachable;

    // drawing

    // camera projection
    const aspect = sapp.widthf() / sapp.heightf();
    const proj = mat4.persp(110.0, aspect, 0.01, 10000.0);

    const vp = proj.mul(state.view);

    // vs params

    // rendering #RENDER #DRAW
    sg.beginPass(.{ .action = state.pass_action, .swapchain = sglue.swapchain() });
    sg.applyPipeline(state.pip);

    for (asset_block.nodes.items) |node| {
        if (node.mesh) |mesh| {
            for (mesh.primitives.items) |primitive| {
                var color = [4]f32{ 1, 1, 1, 1 };

                if (primitive.material) |material| {
                    if (material.texture) |texture| {
                        state.bind.views[shd.VIEW_tex] = texture.view;
                    } else {
                        state.bind.views[shd.VIEW_tex] = state.white_tex_view;
                    }

                    color = material.color;
                } else {
                    state.bind.views[shd.VIEW_tex] = state.white_tex_view;
                    color = [4]f32{ 1, 0, 1, 1 };
                }

                state.bind.vertex_buffers[0] = primitive.vertex_buffer;
                state.bind.index_buffer = primitive.index_buffer;

                sg.applyBindings(state.bind);

                const vs_params = shd.VsParams{
                    .mvp = vp.mul(node.m),
                };

                sg.applyUniforms(shd.UB_vs_params, sg.asRange(&vs_params));

                const fs_params = shd.FsParams{
                    .u_color = color,
                };

                sg.applyUniforms(shd.UB_fs_params, sg.asRange(&fs_params));

                sg.draw(0, primitive.index_count, 1);
            }
        }
    }

    // for (state.cubes.items) |cube| {
    //     drawCube(&vp, cube.pos, cube.size);
    // }

    {
        // #GUI
        var b = true;
        ig.igSetNextWindowSize(.{ .x = 300, .y = 0 }, 0);
        _ = ig.igBegin("window", &b, 0);
        defer ig.igEnd();

        ig.igText("speed kmh: %.2f", speedKmH);

        _ = ig.igCheckbox("use gamepad", &state.useGamepad);

        var strbuf = std.mem.zeroes([64]u8);
        if (state.attachedGamepad) |gpad| {
            ig.igText("Device: %s", gpad.description);

            for (0..gpad.numAxes) |i| {
                var axisValue = gpad.axisStates[i];
                const axisName = std.fmt.bufPrintZ(&strbuf, "axis {}", .{i}) catch unreachable;
                _ = ig.igSliderFloat(axisName.ptr, &axisValue, -1, 1);
            }
        }

        ig.igText("Inputs:");
        _ = ig.igSliderFloat("roll", &rollAccel, -1, 1);
        _ = ig.igSliderFloat("pitch", &pitchAccel, -1, 1);
        _ = ig.igSliderFloat("yaw", &yawAccel, -1, 1);
        _ = ig.igSliderFloat("throttle", &yAccel, 0, 1);
    }

    simgui.render();

    sg.endPass();

    sg.commit();
}

// #INPUT

pub const InputMap = struct {
    const K = sapp.Keycode;

    pub var throttle_up = K.W;
    pub var throttle_down = K.S;
    pub var pitch_up = K.DOWN;
    pub var pitch_down = K.UP;
    pub var roll_left = K.LEFT;
    pub var roll_right = K.RIGHT;
    pub var yaw_left = K.A;
    pub var yaw_right = K.D;
    pub var exit = K.ESCAPE;
};

var input_state = std.enums.EnumFieldStruct(std.meta.DeclEnum(InputMap), bool, false){};

// #INPUT MARK: input()
export fn input(event: ?*const sapp.Event) void {
    const ev = event.?;

    if (simgui.handleEvent(ev.*))
        return;

    inline for (std.meta.fields(@TypeOf(input_state))) |field| {
        const key: sapp.Keycode = @field(InputMap, field.name);
        if (ev.key_code == key) {
            switch (ev.type) {
                .KEY_DOWN => @field(input_state, field.name) = true,
                .KEY_UP => @field(input_state, field.name) = false,
                else => {},
            }
        }
    }

    if (input_state.exit)
        sapp.requestQuit();
}

// MARK: cleanup()
export fn cleanup() void {
    std.log.debug("Cleanup!", .{});

    asset_block.deinit(state.gpa.allocator());
    _ = state.gpa.deinit();

    sg.shutdown();
    phy.deinit();
}

// MARK: Config

fn eql(left: []const u8, right: []const u8) bool {
    return std.mem.eql(u8, left, right);
}

fn find(haystack: []const u8, needle: []const u8) ?usize {
    return std.mem.indexOf(u8, haystack, needle);
}

fn processConfigLine(key: []const u8, value: []const u8) !void {
    const fields = std.meta.fields(AxisBindings);

    if (find(key, "_")) |cati| {
        const category = key[0..cati];
        const afterCat = key[cati + 1 ..];

        if (eql(category, "axis")) {
            if (find(afterCat, "_")) |axisi| {
                const actionName = afterCat[0..axisi];

                inline for (fields) |field| {
                    if (eql(actionName, field.name)) {
                        const suffix = afterCat[axisi + 1 ..];

                        if (eql(suffix, "id")) {
                            @field(axisBindings, field.name).id = try std.fmt.parseInt(u8, value, 10);
                        } else if (eql(suffix, "deadzone")) {
                            @field(axisBindings, field.name).deadzone = try std.fmt.parseFloat(f32, value);
                        }
                    }
                }
            }
        } else if (eql(category, "key")) {
            var buff = std.mem.zeroes([32]u8);
            const val = std.ascii.upperString(&buff, value);

            const keycode = std.meta.stringToEnum(sapp.Keycode, val) orelse sapp.Keycode.INVALID;

            if (keycode == .INVALID) {
                std.log.err("Bad keycode string for '{s}'. '{s}' key doesn't exist", .{ afterCat, val });
            }

            const inputMapDecls = @typeInfo(InputMap).@"struct".decls;

            var found = false;
            inline for (inputMapDecls) |decl| {
                if (eql(decl.name, afterCat)) {
                    @field(InputMap, decl.name) = keycode;
                    found = true;
                }
            }

            if (!found) {
                std.log.err("Binding '{s}' not found", .{afterCat});
            }
        }
    }
}

fn saveConfig() !void {
    if (std.fs.cwd().createFile(configPath, .{})) |file| {
        defer file.close();

        var buff = std.mem.zeroes([1024]u8);
        var writer = file.writer(&buff);

        _ = try ini.saveStruct(configData, &writer.interface);

        try writer.interface.flush();
    } else |err| {
        return err;
    }
}

fn loadConfig() !void {
    if (std.fs.cwd().openFile(configPath, .{})) |file| {
        defer file.close();

        var buff = std.mem.zeroes([1024]u8);
        var reader = file.reader(&buff);

        try ini.loadStruct(&configData, &reader.interface, null);
    } else |err| {
        switch (err) {
            error.FileNotFound => {
                std.log.info(configPath ++ " not found, using defaults", .{});
            },
            else => return err,
        }
    }
}

// MARK: main()
pub fn main() !void {
    const a: i32 = 4;
    const b = &a;
    std.log.info("b: {any}", .{b});

    var window_args = struct {
        fullscreen: bool = false,
    }{};

    var args = std.process.args();
    while (args.next()) |arg| {
        if (eql(arg, "-f") or eql(arg, "--fullscreen")) {
            window_args.fullscreen = true;
        }
        if (eql(arg, "-h") or eql(arg, "--help")) {
            var stdout = std.fs.File.stdout();
            _ = try stdout.write(
                \\ DroneSim - A little FPV racing simulator
                \\      -f --fullscreen   - start in fullscreen
                \\
            );
            return;
        }
    }

    // bindings ini
    {
        const fileOrErr = std.fs.cwd().openFile(bindingsPath, .{});

        if (fileOrErr) |file| {
            defer file.close();

            var buff: [1024]u8 = undefined;
            var reader = file.readerStreaming(&buff);
            var iniIter = ini.EntryReader{ .reader = &reader.interface };

            while (iniIter.next()) |res| {
                try processConfigLine(res.key, res.value);
            }
        } else |err| {
            switch (err) {
                error.FileNotFound => {
                    std.log.info(bindingsPath ++ " not found, using defaults", .{});
                },
                else => return err,
            }
        }
    }

    try loadConfig();

    sapp.run(.{
        .init_cb = init,
        .frame_cb = frame,
        .cleanup_cb = cleanup,
        .event_cb = input,
        .width = 800,
        .height = 600,
        .fullscreen = window_args.fullscreen,
        .sample_count = 4,
        .icon = .{ .sokol_default = true },
        .window_title = "DroneSim",
        .logger = .{ .func = slog.func },
    });
}
