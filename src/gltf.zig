const std = @import("std");
const zgltf = @import("zgltf");
const main = @import("main.zig");
const zigimg = @import("zigimg");
const sg = @import("sokol").gfx;
const math = @import("math.zig");

const AssetBlock = main.AssetBlock;
const Texture = main.Texture;
const Mesh = main.Mesh;
const Primitive = main.Primitive;
const Node = main.Node;
const MeshData = main.MeshData;
const Material = main.Material;

pub fn load(alloc: std.mem.Allocator, gltf_buffer: []align(4) const u8) !AssetBlock {
    var gltf = zgltf.Gltf.init(alloc);
    defer gltf.deinit();

    std.log.info("------------ LOADING GLTF -----------", .{});

    // Load from buffer

    try gltf.parse(gltf_buffer);

    var asset_block: AssetBlock = undefined;
    try asset_block.init(alloc);

    // Image / Texture

    for (gltf.data.images) |gltf_image| {
        const image = try zigimg.Image.fromMemory(alloc, gltf_image.data.?);

        const bytes = image.pixels.asConstBytes(); // try alloc.dupe(u8, image.pixels.asConstBytes());

        std.log.info("Image {}: {s}", .{ asset_block.textures.items.len, gltf_image.name orelse "NO NAME" });
        std.log.info("     -- width {}, height {}, format {}", .{ image.width, image.height, image.pixelFormat() });

        const image_view = sg.makeView(.{
            .texture = .{
                .image = sg.makeImage(.{
                    .width = @intCast(image.width),
                    .height = @intCast(image.height),
                    .data = init: {
                        var data = sg.ImageData{};
                        data.mip_levels[0] = sg.asRange(bytes);
                        break :init data;
                    },
                }),
            },
        });

        // state.bind.views[shd.VIEW_tex] = image_view;

        try asset_block.textures.append(alloc, .{
            .image = image,
            .view = image_view,
        });

        std.log.info("     -- view id: {}, bytes ptr {*}", .{ image_view.id, bytes.ptr });
    }

    // Material

    for (gltf.data.materials) |gltfMaterial| {
        std.log.info("", .{});
        std.log.info("Material: \"{s}\"", .{gltfMaterial.name.?});

        const col = gltfMaterial.metallic_roughness.base_color_factor;
        std.log.info("   - color {any}", .{col});

        var tex: ?*Texture = null;

        if (gltfMaterial.metallic_roughness.base_color_texture) |gltfTexture| {
            std.debug.assert(asset_block.textures.items.len > gltfTexture.index);

            tex = &asset_block.textures.items[gltfTexture.index];
            std.log.info("   - has color texture! Index: {}", .{gltfTexture.index});
        } else {
            std.log.info("   - no texture", .{});
        }

        try asset_block.materials.append(alloc, .{
            .texture = tex,
            .color = col,
        });
    }

    // Mesh / Primitive

    for (gltf.data.meshes) |gltf_mesh| {
        // const mesh_ptr = try GLTFState.meshes.addOne(alloc);
        // mesh_ptr.* = .init();

        try asset_block.meshes.append(alloc, try .init(alloc));
        const mesh_ptr = &asset_block.meshes.items[asset_block.meshes.items.len - 1];

        std.debug.assert(mesh_ptr.primitives.items.len == 0);

        for (gltf_mesh.primitives) |gltf_primitive| {
            var mesh_data = try alloc.create(MeshData);

            mesh_data.* = .{
                .vertices = try .initCapacity(alloc, 1024),
                .indices = try .initCapacity(alloc, 1024),
            };

            const vertices = &mesh_data.vertices;
            const indices = &mesh_data.indices;

            for (gltf_primitive.attributes) |attribute| {
                switch (attribute) {
                    .position => |accessor_index| {
                        const accessor = gltf.data.accessors[accessor_index];
                        const view = try gltf.getDataFromBufferView(f32, alloc, accessor, gltf.glb_binary.?);
                        defer alloc.free(view);

                        std.debug.assert(accessor.component_type == .float);
                        std.debug.assert(accessor.type == .vec3);

                        const vertexCount: usize = @intCast(accessor.count);

                        try vertices.ensureTotalCapacity(alloc, vertexCount);

                        std.log.info("    -- VERTICES count: {}", .{vertexCount});

                        for (0..vertexCount) |vertexIndex| {
                            vertices.appendAssumeCapacity(.{
                                .x = view[vertexIndex * 3 + 0],
                                .y = view[vertexIndex * 3 + 1],
                                .z = view[vertexIndex * 3 + 2],
                                .color = 0xFFFFFFFF,
                                .u = 0,
                                .v = 0,
                            });
                        }
                    },
                    .texcoord => |accessor_index| {
                        const accessor = gltf.data.accessors[accessor_index];

                        std.debug.assert(accessor.component_type == .float);
                        std.debug.assert(accessor.type == .vec2);

                        const view = try gltf.getDataFromBufferView(f32, alloc, accessor, gltf.glb_binary.?);
                        defer alloc.free(view);

                        std.log.info("      -- uvs: {} == {} ?", .{ vertices.items.len, accessor.count });

                        std.debug.assert(vertices.items.len > 0);
                        std.debug.assert(view.len == vertices.items.len * 2);

                        for (vertices.items, 0..) |*vertex, i| {
                            vertex.u = @intFromFloat(view[i * 2 + 0] * 32767);
                            vertex.v = @intFromFloat(view[i * 2 + 1] * 32767);
                        }
                    },
                    else => {},
                }
            }

            const accessor = gltf.data.accessors[gltf_primitive.indices.?];
            if (accessor.component_type == .unsigned_short) {
                const view = try gltf.getDataFromBufferView(u16, alloc, accessor, gltf.glb_binary.?);
                defer alloc.free(view);

                try indices.ensureTotalCapacity(alloc, view.len);

                std.log.info("    -- INDICES: count: {}, triangles: {}, type: short", .{ view.len, @divExact(view.len, 3) });

                var i: usize = 0;
                while (i < view.len) : (i += 3) {
                    indices.appendAssumeCapacity(@intCast(view[i + 1]));
                    indices.appendAssumeCapacity(@intCast(view[i + 0]));
                    indices.appendAssumeCapacity(@intCast(view[i + 2]));
                }

                // for (intView) |vi| {
                //     try indices.append(alloc, @intCast(vi));
                // }
            } else if (accessor.component_type == .unsigned_integer) {
                @panic("u32 indices are not supported");
            }

            const material: ?*Material = if (gltf_primitive.material) |mati|
                &asset_block.materials.items[mati]
            else
                null;

            mesh_ptr.primitives.appendAssumeCapacity(.{
                .vertex_buffer = sg.makeBuffer(.{
                    .data = sg.asRange(vertices.items),
                }),
                .index_buffer = sg.makeBuffer(
                    .{
                        .data = sg.asRange(indices.items),
                        .usage = .{ .index_buffer = true },
                    },
                ),
                .index_count = @intCast(indices.items.len + 9),
                .data = mesh_data,
                .material = material,
            });

            std.debug.assert(mesh_data.vertices.items.len == vertices.items.len);
        } // for primitives
    } // for meshes

    // Node

    for (gltf.data.nodes) |gltf_node| {
        var node: Node = .{};

        if (gltf_node.name) |name| {
            node.name = try alloc.dupeZ(u8, name);
        }

        std.log.info("Node: '{s}'", .{node.name});

        if (gltf_node.matrix) |mat| {
            node.m.m = .{ mat[0..4].*, mat[4..8].*, mat[8..12].*, mat[12..16].* };
            std.log.info("   -- found model matrix", .{});
        } else {
            const mat4 = math.Mat4;
            const vec3 = math.Vec3;

            const pos = mat4.translate(vec3.fromArr(gltf_node.translation));
            const rot = mat4.rotateByQuat(gltf_node.rotation);
            const scale = mat4.scale(vec3.fromArr(gltf_node.scale));

            node.m = pos.mul(rot).mul(scale);

            std.log.info("   -- pos: {any}, rot: {any}, scl: {any}", .{
                gltf_node.translation,
                gltf_node.rotation,
                gltf_node.scale,
            });
        }

        if (gltf_node.mesh) |meshi| {
            node.mesh = &asset_block.meshes.items[meshi];
            std.log.info("   -- Has mesh index: {}", .{meshi});
        } else {
            std.log.info("   -- Not a mesh", .{});
        }

        try asset_block.nodes.append(alloc, node);
    }

    std.log.info("------------ Finished loading GLTF -----------", .{});

    std.log.info("Vert 0: {any}", .{asset_block.nodes.items[0].mesh.?.primitives.items[0].data.?.vertices.items[0]});

    return asset_block;
}

pub fn deinit(asset_block: *AssetBlock, alloc: std.mem.Allocator) void {
    for (asset_block.textures.items) |*texture| {
        texture.image.deinit(alloc);
        std.log.info("deiniting texture", .{});
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

    asset_block.deinit(alloc);
}
