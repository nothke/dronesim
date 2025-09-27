const sokol = @import("sokol");
const slog = sokol.log;
const sg = sokol.gfx;
const sapp = sokol.app;
const sglue = sokol.glue;
const vec3 = @import("math.zig").Vec3;
const mat4 = @import("math.zig").Mat4;
const shd = @import("shaders/texcube.glsl.zig");
const std = @import("std");
const phy = @import("zphysics");
const ig = @import("cimgui");
const simgui = sokol.imgui;

const c = @cImport({
    @cInclude("Gamepad.h");
});

const max_cubes = 1024;
const max_bodies = 1024;

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

    var attachedGamepad: ?*c.struct_Gamepad_device = null;

    var gamepadInputThrottle: f32 = -1;

    var iterationsToNextGamepadPoll: u8 = 0;
    const iterationsToWaitForGamepadPoll = 60;

    var useGamepad = true;
};

const WorldCube = struct {
    pos: vec3,
    size: vec3,
    bodyId: phy.BodyId,
};

// a vertex struct with position, color and uv-coords
const Vertex = extern struct { x: f32, y: f32, z: f32, color: u32, u: i16, v: i16 };

// Physics structs

const object_layers = struct {
    const non_moving: phy.ObjectLayer = 0;
    const moving: phy.ObjectLayer = 1;
    const len: u32 = 2;
};

const broad_phase_layers = struct {
    const non_moving: phy.BroadPhaseLayer = 0;
    const moving: phy.BroadPhaseLayer = 1;
    const len: u32 = 2;
};

const BroadPhaseLayerInterface = extern struct {
    broad_phase_layer_interface: phy.BroadPhaseLayerInterface = .init(@This()),
    object_to_broad_phase: [object_layers.len]phy.BroadPhaseLayer = undefined,

    fn init() BroadPhaseLayerInterface {
        var object_to_broad_phase: [object_layers.len]phy.BroadPhaseLayer = undefined;
        object_to_broad_phase[object_layers.non_moving] = broad_phase_layers.non_moving;
        object_to_broad_phase[object_layers.moving] = broad_phase_layers.moving;
        return .{ .object_to_broad_phase = object_to_broad_phase };
    }

    fn selfPtr(broad_phase_layer_interface: *phy.BroadPhaseLayerInterface) *BroadPhaseLayerInterface {
        return @alignCast(@fieldParentPtr("broad_phase_layer_interface", broad_phase_layer_interface));
    }

    fn selfPtrConst(broad_phase_layer_interface: *const phy.BroadPhaseLayerInterface) *const BroadPhaseLayerInterface {
        return @alignCast(@fieldParentPtr("broad_phase_layer_interface", broad_phase_layer_interface));
    }

    pub fn getNumBroadPhaseLayers(_: *const phy.BroadPhaseLayerInterface) callconv(.c) u32 {
        return broad_phase_layers.len;
    }

    pub fn getBroadPhaseLayer(
        broad_phase_layer_interface: *const phy.BroadPhaseLayerInterface,
        layer: phy.ObjectLayer,
    ) callconv(.c) phy.BroadPhaseLayer {
        return selfPtrConst(broad_phase_layer_interface).object_to_broad_phase[layer];
    }
};

const ObjectVsBroadPhaseLayerFilter = extern struct {
    object_vs_broad_phase_layer_filter: phy.ObjectVsBroadPhaseLayerFilter = .init(@This()),

    pub fn shouldCollide(
        _: *const phy.ObjectVsBroadPhaseLayerFilter,
        layer1: phy.ObjectLayer,
        layer2: phy.BroadPhaseLayer,
    ) callconv(.c) bool {
        return switch (layer1) {
            object_layers.non_moving => layer2 == broad_phase_layers.moving,
            object_layers.moving => true,
            else => unreachable,
        };
    }
};

const ObjectLayerPairFilter = extern struct {
    object_layer_pair_filter: phy.ObjectLayerPairFilter = .init(@This()),

    pub fn shouldCollide(
        _: *const phy.ObjectLayerPairFilter,
        object1: phy.ObjectLayer,
        object2: phy.ObjectLayer,
    ) callconv(.c) bool {
        return switch (object1) {
            object_layers.non_moving => object2 == object_layers.moving,
            object_layers.moving => true,
            else => unreachable,
        };
    }
};

const ContactListener = extern struct {
    contact_listener: phy.ContactListener = .init(@This()),

    fn selfPtr(contact_listener: *phy.ContactListener) *ContactListener {
        return @alignCast(@fieldParentPtr("contact_listener", contact_listener));
    }

    fn selfPtrConst(contact_listener: *const phy.ContactListener) *const ContactListener {
        return @alignCast(@fieldParentPtr("contact_listener", contact_listener));
    }

    pub fn onContactValidate(
        contact_listener: *phy.ContactListener,
        body1: *const phy.Body,
        body2: *const phy.Body,
        base_offset: *const [3]phy.Real,
        collision_result: *const phy.CollideShapeResult,
    ) callconv(.c) phy.ValidateResult {
        _ = contact_listener;
        _ = body1;
        _ = body2;
        _ = base_offset;
        _ = collision_result;
        return .accept_all_contacts;
    }

    pub fn onContactAdded(
        contact_listener: *phy.ContactListener,
        body1: *const phy.Body,
        body2: *const phy.Body,
        _: *const phy.ContactManifold,
        _: *phy.ContactSettings,
    ) callconv(.c) void {
        _ = contact_listener;
        _ = body1;
        _ = body2;
    }

    pub fn onContactPersisted(
        contact_listener: *phy.ContactListener,
        body1: *const phy.Body,
        body2: *const phy.Body,
        _: *const phy.ContactManifold,
        _: *phy.ContactSettings,
    ) callconv(.c) void {
        _ = contact_listener;
        _ = body1;
        _ = body2;
    }

    pub fn onContactRemoved(
        contact_listener: *phy.ContactListener,
        sub_shape_id_pair: *const phy.SubShapeIdPair,
    ) callconv(.c) void {
        _ = contact_listener;
        _ = sub_shape_id_pair;
    }
};

// Nothkes physics

fn createBoxBody(body_interface: *phy.BodyInterface, size: vec3, pos: vec3, moving: bool) !phy.BodyId {
    const settings = try phy.BoxShapeSettings.create(.{ size.x, size.y, size.z });
    defer settings.asShapeSettings().release();

    const shape = try settings.asShapeSettings().createShape();
    defer shape.release();

    return try body_interface.createAndAddBody(
        .{
            .position = .{ pos.x, pos.y, pos.z, 0 },
            .rotation = .{ 0, 0, 0, 1 },
            .shape = shape,
            .motion_type = if (moving) .dynamic else .static,
            .object_layer = if (moving) object_layers.moving else object_layers.non_moving,
            .allow_sleeping = false,
        },
        .activate,
    );
}

// End of physics

fn createBox(body_interface: *phy.BodyInterface, pos: vec3, size: vec3) void {
    const bodyId = createBoxBody(body_interface, size, pos, false) catch unreachable;
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
    if (axisId == 5)
        state.gamepadInputThrottle = value;
    // std.log.info("axis moved: {}, went from: {} to: {}", .{ axisId, lastValue, value });
}

// #INIT
export fn init() void {
    sg.setup(.{
        .environment = sglue.environment(),
        .logger = .{ .func = slog.func },
    });

    simgui.setup(.{
        .logger = .{ .func = slog.func },
    });

    // gamepad input

    c.Gamepad_deviceAttachFunc(gamepadOnDeviceAttached, null);
    c.Gamepad_deviceRemoveFunc(gamepadOnDeviceDetached, null);
    c.Gamepad_axisMoveFunc(gamepadOnAxisMove, null);
    c.Gamepad_init();

    const cs: f32 = 1;

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
        }),
        // zig fmt: on
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

    // physics

    const alloc = std.heap.page_allocator;

    try phy.init(alloc, .{});

    const broadphase_layer_interface = alloc.create(BroadPhaseLayerInterface) catch unreachable;
    broadphase_layer_interface.* = BroadPhaseLayerInterface.init();

    const object_vs_broad_phase_layer_filter = alloc.create(ObjectVsBroadPhaseLayerFilter) catch unreachable;
    object_vs_broad_phase_layer_filter.* = .{};

    const object_layer_pair_filter = alloc.create(ObjectLayerPairFilter) catch unreachable;
    object_layer_pair_filter.* = .{};

    const contact_listener = alloc.create(ContactListener) catch unreachable;
    contact_listener.* = .{};

    state.physics_system = phy.PhysicsSystem.create(
        @as(*const phy.BroadPhaseLayerInterface, @ptrCast(broadphase_layer_interface)),
        @as(*const phy.ObjectVsBroadPhaseLayerFilter, @ptrCast(object_vs_broad_phase_layer_filter)),
        @as(*const phy.ObjectLayerPairFilter, @ptrCast(object_layer_pair_filter)),
        .{
            .max_bodies = max_bodies,
            .num_body_mutexes = 0,
            .max_body_pairs = 1024,
            .max_contact_constraints = 1024,
        },
    ) catch unreachable;

    defer state.physics_system.optimizeBroadPhase();

    const body_interface = state.physics_system.getBodyInterfaceMut();

    // physics spawning

    // #DRONEINIT
    state.droneBodyId = createBoxBody(body_interface, vec3.new(0.1, 0.05, 0.1), vec3.new(0, 1, 0), true) catch unreachable;

    state.cubes = std.ArrayListUnmanaged(WorldCube).initBuffer(&state.cubesBuffer);

    createBox(body_interface, vec3.zero(), vec3.new(1000, 1, 1000));
    createBox(body_interface, vec3.new(0, 5, 10), vec3.new(1, 10, 1));
    createBox(body_interface, vec3.new(0, 5, -10), vec3.new(1, 10, 1));
    createBox(body_interface, vec3.new(5, 5, -20), vec3.new(1, 10, 1));
    createBox(body_interface, vec3.new(-5, 5, -30), vec3.new(1, 10, 1));

    var pcg = std.Random.Pcg.init(234583423);
    const r = pcg.random();

    const range = 1000;

    for (0..800) |_| {
        createBox(
            body_interface,
            vec3.new(-500 + r.float(f32) * range, 20 + r.float(f32) * 20, -500 + r.float(f32) * range),
            vec3.new(1 + r.float(f32) * 6, 50 + r.float(f32) * 50, 1 + r.float(f32) * 6),
        );
    }
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
    // TODO: Move to math
    const scale = mat4{ .m = .{
        .{ size.x, 0, 0, 0 },
        .{ 0, size.y, 0, 0 },
        .{ 0, 0, size.z, 0 },
        .{ 0, 0, 0, 1 },
    } };

    const model = mat4.translate(pos).mul(scale);

    const vs_params = shd.VsParams{ .mvp = vp.mul(model) };

    sg.applyUniforms(shd.UB_vs_params, sg.asRange(&vs_params));
    sg.draw(0, 36, 1);
}

// #LOOP
export fn frame() void {
    simgui.newFrame(.{
        .width = sapp.width(),
        .height = sapp.height(),
        .delta_time = sapp.frameDuration(),
        .dpi_scale = 1,
    });

    const dt: f32 = @floatCast(sapp.frameDuration());

    // Move to mat4
    var dUp = vec3.new(state.view.m[0][1], state.view.m[1][1], state.view.m[2][1]);
    //const dRight = vec3.new(state.view.m[0][0], state.view.m[1][0], state.view.m[2][0]);
    const dForward = vec3.new(state.view.m[0][2], state.view.m[1][2], state.view.m[2][2]);

    dUp = dUp.add(dForward.mul(-0.7)).norm();

    // input

    if (state.iterationsToNextGamepadPoll > state.iterationsToWaitForGamepadPoll) {
        state.iterationsToNextGamepadPoll = 0;
        c.Gamepad_detectDevices();
    }
    state.iterationsToNextGamepadPoll += 1;

    c.Gamepad_processEvents();

    var yAccel: f32 = keyAxisInput(false, input_state.throttleUp);
    var pitchAccel: f32 = keyAxisInput(input_state.pitchDown, input_state.pitchUp);
    var rollAccel: f32 = keyAxisInput(input_state.rollLeft, input_state.rollRight);
    var yawAccel: f32 = keyAxisInput(input_state.yawLeft, input_state.yawRight);

    if (state.useGamepad) {
        if (state.attachedGamepad) |gpad| {
            yAccel = (1 + state.gamepadInputThrottle) * 0.5; // gpad.axisStates[5]
            yawAccel = axisInput(gpad.axisStates[0], 0.2);
            rollAccel = axisInput(gpad.axisStates[3], 0.2);
            pitchAccel = axisInput(gpad.axisStates[4], 0.2);
        }
    }

    yAccel = std.math.clamp(yAccel, 0, 1);
    pitchAccel = std.math.clamp(pitchAccel, -1, 1);
    rollAccel = std.math.clamp(rollAccel, -1, 1);
    yawAccel = std.math.clamp(yawAccel, -1, 1);

    // physics

    const mutBodies = state.physics_system.getBodiesMutUnsafe();

    for (mutBodies) |body| {
        if (!phy.isValidBodyPointer(body) or body.motion_properties == null) continue;

        if (body.id == state.droneBodyId) {
            // #DRONEUPDATE
            const thrustForceMult = 200;
            const rollPitchTorqueMult = 0.5;
            const yawTorqueMult = 0.1;

            const upForce = vec3.mul(dUp, yAccel * thrustForceMult);
            body.addForce(upForce.asArr());
            body.addTorque(.{
                -rollPitchTorqueMult * pitchAccel,
                yawTorqueMult * yawAccel,
                rollPitchTorqueMult * rollAccel,
            });

            const dragMult: f32 = 2.0;
            const angularDragMult: f32 = 0.5;
            body.applyBuoyancyImpulse(.{ 0, body.position[1] + 100, 0 }, .{ 0, 1, 0 }, 0.01, dragMult, angularDragMult, .{ 0, 0, 0 }, .{ 0, -9.81, 0 }, dt);
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

            // TODO: Move to mat4
            const rotMat = mat4{ .m = .{
                .{ r[0], r[1], r[2], 0 },
                .{ r[3], r[4], r[5], 0 },
                .{ r[6], r[7], r[8], 0 },
                .{ 0, 0, 0, 1 },
            } };

            v = v.mul(rotMat);
            v = v.mul(mat4.translate(vec3.new(dpos[0], dpos[1], dpos[2]).mul(-1)));

            state.view = v;
        }
    }

    state.physics_system.update(dt, .{}) catch unreachable;

    // drawing

    // camera projection
    const aspect = sapp.widthf() / sapp.heightf();
    const proj = mat4.persp(110.0, aspect, 0.01, 10000.0);

    const vp = proj.mul(state.view);

    // vs params

    // rendering
    sg.beginPass(.{ .action = state.pass_action, .swapchain = sglue.swapchain() });
    sg.applyPipeline(state.pip);
    sg.applyBindings(state.bind);

    for (state.cubes.items) |cube| {
        drawCube(&vp, cube.pos, cube.size);
    }

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

    pub const throttleUp = K.W;
    pub const throttleDown = K.S;
    pub const pitchUp = K.DOWN;
    pub const pitchDown = K.UP;
    pub const rollLeft = K.LEFT;
    pub const rollRight = K.RIGHT;
    pub const yawLeft = K.A;
    pub const yawRight = K.D;
    pub const exit = K.ESCAPE;
};

var input_state = std.enums.EnumFieldStruct(std.meta.DeclEnum(InputMap), bool, false){};

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

export fn cleanup() void {
    sg.shutdown();
    phy.deinit();
}

pub fn main() void {
    sapp.run(.{
        .init_cb = init,
        .frame_cb = frame,
        .cleanup_cb = cleanup,
        .event_cb = input,
        .width = 800,
        .height = 600,
        .fullscreen = false,
        .sample_count = 4,
        .icon = .{ .sokol_default = true },
        .window_title = "DroneSim",
        .logger = .{ .func = slog.func },
    });
}
