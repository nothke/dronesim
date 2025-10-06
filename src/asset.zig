const std = @import("std");
const zgltf = @import("zgltf");
const main = @import("main.zig");
const zigimg = @import("zigimg");
const sg = @import("sokol").gfx;

const math = @import("math.zig");
const vec3 = math.Vec3;
const mat4 = math.Mat4;

// a vertex struct with position, color and uv-coords
pub const Vertex = extern struct {
    x: f32,
    y: f32,
    z: f32,
    color: u32,
    u: i16,
    v: i16,
};

pub const Texture = struct {
    image: zigimg.Image,
    view: sg.View,
};

pub const Material = struct {
    color: [4]f32,
    texture: ?*Texture,

    pub fn black() Material {
        return .{ .color = .{ 0, 0, 0, 1 }, .texture = null };
    }

    pub fn white() Material {
        return .{ .color = .{ 1, 1, 1, 1 }, .texture = null };
    }
};

pub const MeshData = struct {
    vertices: std.ArrayList(Vertex),
    indices: std.ArrayList(u16),

    fn init(alloc: std.mem.Allocator, vertices_capacity: usize, indices_capacity: usize) MeshData {
        return .{
            .indices = .initCapacity(alloc, indices_capacity),
            .vertices = .initCapacity(alloc, vertices_capacity),
        };
    }
};

pub const Primitive = struct {
    material: ?*Material = null,
    index_count: u32 = 0,
    vertex_buffer: sg.Buffer = .{},
    index_buffer: sg.Buffer = .{},
    data: ?*MeshData = null,

    pub fn freeMeshData(self: *Primitive, alloc: std.mem.Allocator) void {
        if (self.data) |data_ptr| {
            data_ptr.vertices.deinit(alloc);
            data_ptr.indices.deinit(alloc);

            alloc.destroy(data_ptr);

            self.data = null;
        }
    }
};

pub const Mesh = struct {
    primitives: std.ArrayList(Primitive) = undefined,

    pub fn init(alloc: std.mem.Allocator) !Mesh {
        return Mesh{
            .primitives = try .initCapacity(alloc, 8),
        };
    }

    pub fn deinit(self: *Mesh, alloc: std.mem.Allocator) void {
        self.primitives.deinit(alloc);
    }

    fn setMaterial(self: *Mesh, material: *Material) void {
        for (self.primitives.items) |*prim| {
            prim.material = material;
        }
    }
};

pub const Node = struct {
    name: [:0]const u8 = "",
    m: mat4 = .identity(),
    // if node has no mesh, its a dummy
    mesh: ?*Mesh = null,

    pub fn freeName(self: *Node, alloc: std.mem.Allocator) void {
        if (self.name.len != 0) {
            alloc.free(self.name);
            self.name = "";
        }
    }
};

pub const AssetBlock = struct {
    nodes: std.ArrayList(Node) = undefined,

    meshes: std.ArrayList(Mesh) = undefined,
    textures: std.ArrayList(Texture) = undefined,
    materials: std.ArrayList(Material) = undefined,

    pub fn init(self: *AssetBlock, alloc: std.mem.Allocator) !void {
        self.nodes = try .initCapacity(alloc, 64);
        self.meshes = try .initCapacity(alloc, 64);
        self.textures = try .initCapacity(alloc, 64);
        self.materials = try .initCapacity(alloc, 64);
    }

    /// Only deinits top-level lists
    pub fn deinitLists(self: *AssetBlock, alloc: std.mem.Allocator) void {
        self.nodes.deinit(alloc);
        self.meshes.deinit(alloc);
        self.textures.deinit(alloc);
        self.materials.deinit(alloc);
    }

    /// Full deinit of all assets
    pub fn deinit(asset_block: *AssetBlock, alloc: std.mem.Allocator) void {
        for (asset_block.textures.items) |*texture| {
            texture.image.deinit(alloc);
        }

        for (asset_block.nodes.items) |*node| {
            node.freeName(alloc);
        }

        for (asset_block.meshes.items) |*mesh| {
            for (mesh.primitives.items) |*primitive| {
                primitive.freeMeshData(alloc);
            }

            mesh.primitives.deinit(alloc);
        }

        asset_block.deinitLists(alloc);
    }
};
