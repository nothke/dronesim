const std = @import("std");
const phy = @import("zphysics");
const vec3 = @import("math.zig").Vec3;
const asset = @import("asset.zig");

// Jolt

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

pub const BroadPhaseLayerInterface = extern struct {
    broad_phase_layer_interface: phy.BroadPhaseLayerInterface = .init(@This()),
    object_to_broad_phase: [object_layers.len]phy.BroadPhaseLayer = undefined,

    pub fn init() BroadPhaseLayerInterface {
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

pub const ObjectVsBroadPhaseLayerFilter = extern struct {
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

pub const ObjectLayerPairFilter = extern struct {
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

pub const ContactListener = extern struct {
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

// dronesim

pub fn createBoxBody(body_interface: *phy.BodyInterface, size: vec3, pos: vec3, moving: bool) !phy.BodyId {
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

pub fn createMeshCollider(mesh_data: *const asset.MeshData) !*phy.Shape {
    std.debug.assert(mesh_data.vertices.items.len > 0);
    std.debug.assert(mesh_data.indices.items.len > 0);

    const settings = try phy.MeshShapeSettings.create(
        mesh_data.vertices.items.ptr,
        @intCast(mesh_data.vertices.items.len),
        @sizeOf(asset.Vertex),
        mesh_data.indices.items,
    );
    defer settings.asShapeSettings().release();

    const shape = try settings.asShapeSettings().createShape();
    //defer shape.release();
    return shape;
}

pub fn addStaticBody(
    body_interface: *phy.BodyInterface,
    shape: *phy.Shape,
    pos: vec3,
    rot: [4]f32,
) !phy.BodyId {
    return try body_interface.createAndAddBody(.{
        .position = .{ pos.x, pos.y, pos.z, 0 },
        .rotation = rot,
        .shape = shape,
        .motion_type = .static,
        .object_layer = object_layers.non_moving,
    }, .activate);
}
