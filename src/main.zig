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

const WorldCube = struct {
    pos: vec3,
    size: vec3,
    bodyId: phy.BodyId,
};

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

    var cubesBuffer: [64]WorldCube = undefined;
    var cubes: std.ArrayListUnmanaged(WorldCube) = .{};
};

const input_state = struct {
    var upPressed: bool = false;
    var downPressed: bool = false;
    var leftPressed: bool = false;
    var rightPressed: bool = false;
    var pitchUpPressed: bool = false;
    var pitchDownPressed: bool = false;
    var rollRightPressed: bool = false;
    var rollLeftPressed: bool = false;
    var yawRightPressed: bool = false;
    var yawLeftPressed: bool = false;
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
    const floor_shape_settings = try phy.BoxShapeSettings.create(.{ size.x, size.y, size.z });
    defer floor_shape_settings.asShapeSettings().release();

    const floor_shape = try floor_shape_settings.asShapeSettings().createShape();
    defer floor_shape.release();

    return try body_interface.createAndAddBody(.{
        .position = .{ pos.x, pos.y, pos.z, 0 },
        .rotation = .{ 0, 0, 0, 1 },
        .shape = floor_shape,
        .motion_type = if (moving) .dynamic else .static,
        .object_layer = if (moving) object_layers.moving else object_layers.non_moving,
        .allow_sleeping = false,
    }, .activate);
}

// End of physics

fn createBox(body_interface: *phy.BodyInterface, pos: vec3, size: vec3) void {
    const bodyId = createBoxBody(body_interface, size, pos, false) catch unreachable;
    state.cubes.appendAssumeCapacity(.{ .pos = pos, .size = size, .bodyId = bodyId });
}

export fn init() void {
    sg.setup(.{
        .environment = sglue.environment(),
        .logger = .{ .func = slog.func },
    });

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
        .max_bodies = 1024,
        .num_body_mutexes = 0,
        .max_body_pairs = 1024,
        .max_contact_constraints = 1024,
    },
    ) catch unreachable;

    const body_interface = state.physics_system.getBodyInterfaceMut();

    state.droneBodyId = createBoxBody(body_interface, vec3.new(0.5, 0.5, 0.5), vec3.new(0, 10, 0), true) catch unreachable;

    state.physics_system.optimizeBroadPhase();

    state.cubes = std.ArrayListUnmanaged(WorldCube).initBuffer(&state.cubesBuffer);

    createBox(body_interface, vec3.zero(), vec3.new(100, 1, 100));
    createBox(body_interface, vec3.new(0, 5, 10), vec3.new(1, 10, 1));
    createBox(body_interface, vec3.new(0, 5, -10), vec3.new(1, 10, 1));
    createBox(body_interface, vec3.new(5, 5, -20), vec3.new(1, 10, 1));
    createBox(body_interface, vec3.new(-5, 5, -30), vec3.new(1, 10, 1));

}

fn rawInputAxis(positive: bool, negative: bool) f32 {
    return if (positive) 1 else (if (negative) -1 else 0);
}

fn drawCube(vp: *const mat4, pos: vec3, size: vec3) void {
    const scale = mat4{.m = .{
        .{size.x,0,0,0}, 
        .{0,size.y,0,0}, 
        .{0,0,size.z,0},
        .{0,0,0,1},
    }};

    const model = mat4.translate(pos).mul(scale);

    const vs_params = shd.VsParams{ .mvp = vp.mul(model) };

    sg.applyUniforms(shd.UB_vs_params, sg.asRange(&vs_params));
    sg.draw(0, 36, 1);
}

export fn frame() void {
    const dt: f32 = @floatCast(sapp.frameDuration());

    const dUp = vec3.new(state.view.m[0][1], state.view.m[1][1], state.view.m[2][1]);
    //const dRight = vec3.new(state.view.m[0][0], state.view.m[1][0], state.view.m[2][0]);
    //const dForward = vec3.new(state.view.m[0][2], state.view.m[1][2], state.view.m[2][2]);

    const yAccel: f32 = rawInputAxis(input_state.upPressed, false);
    const pitchAccel: f32 = rawInputAxis(input_state.pitchDownPressed, input_state.pitchUpPressed);
    const rollAccel: f32 = rawInputAxis(input_state.rollLeftPressed, input_state.rollRightPressed);
    const yawAccel: f32 = rawInputAxis(input_state.yawLeftPressed, input_state.yawRightPressed);

    // physics

    const mutBodies = state.physics_system.getBodiesMutUnsafe();

    for (mutBodies) |body| {
        if (!phy.isValidBodyPointer(body) or body.motion_properties == null) continue;

        if (body.motion_type == .dynamic)
        {
            const upForce = vec3.mul(dUp, yAccel * 20000);
            body.addForce(upForce.asArr());
            body.addTorque(.{1000 * pitchAccel, -1000 * yawAccel, -1000 * rollAccel});

            body.applyBuoyancyImpulse(.{0,100,0}, .{0,1,0}, 0, 100, 1, .{0,0,0}, .{0,-9.81,0}, dt);
        }
    }

    const bodies = state.physics_system.getBodiesUnsafe();

    for (bodies) |body| {
        if (!phy.isValidBodyPointer(body) or body.motion_properties == null) continue;

        if(body.motion_type == .dynamic) {
            var v = mat4.identity();
            const dpos = body.getWorldTransform().position;
            
            const r = body.getWorldTransform().rotation;

            // TODO: Move to mat4
            const rotMat = mat4{.m = .{
                .{r[0], r[1], r[2], 0},
                .{r[3], r[4], r[5], 0},
                .{r[6], r[7], r[8], 0},
                .{0,0,0,1},
                }};

            v = v.mul(rotMat);
            v = v.mul(mat4.translate(vec3.new(dpos[0], dpos[1], dpos[2]).mul(-1)));


            state.view = v;
        }
    }

    state.physics_system.update(dt, .{}) catch unreachable;

    // drawing

    // camera projection
    const aspect = sapp.widthf() / sapp.heightf();
    const proj = mat4.persp(90.0, aspect, 0.01, 1000.0);

    const vp = proj.mul(state.view);
    
    // vs params

    // rendering
    sg.beginPass(.{ .action = state.pass_action, .swapchain = sglue.swapchain() });
    sg.applyPipeline(state.pip);
    sg.applyBindings(state.bind);

    for (state.cubes.items) |cube| {
        drawCube(&vp, cube.pos, cube.size);
    }

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

            .UP => input_state.pitchDownPressed = true,
            .DOWN => input_state.pitchUpPressed = true,

            .LEFT => input_state.rollLeftPressed = true,
            .RIGHT => input_state.rollRightPressed = true,

            .A => input_state.yawLeftPressed = true,
            .D => input_state.yawRightPressed = true,
            else => {},
        }
    }

    if (ev.type == .KEY_UP) {
        switch (ev.key_code) {
            .W => input_state.upPressed = false,
            .S =>  input_state.upPressed = false,

            .UP => input_state.pitchDownPressed = false,
            .DOWN => input_state.pitchUpPressed = false,

            .LEFT => input_state.rollLeftPressed = false,
            .RIGHT => input_state.rollRightPressed = false,

            .A => input_state.yawLeftPressed = false,
            .D => input_state.yawRightPressed = false,
            else => {},
        }
    }
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
        .sample_count = 4,
        .icon = .{ .sokol_default = true },
        .window_title = "texcube.zig",
        .logger = .{ .func = slog.func },
    });
}