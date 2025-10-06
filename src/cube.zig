const std = @import("std");
const main = @import("main.zig");

const sg = @import("sokol").gfx;
const Mesh = main.Mesh;
const Vertex = main.Vertex;

pub fn createCubeMesh(alloc: std.mem.Allocator) !Mesh {
    const cs: f32 = 1;

    var mesh = try Mesh.init(alloc);
    const prim_ptr = mesh.primitives.addOneAssumeCapacity();
    // prim_ptr.data = alloc.create(MeshData);
    // const data_ptr = prim_ptr.data.?;
    // data_ptr.* = MeshData.init(alloc, 24, 36);

    prim_ptr.vertex_buffer = sg.makeBuffer(.{
        .data = sg.asRange(&[_]Vertex{
            // zig fmt: off
            .{ .x = -cs, .y = -1.0, .z = -cs, .color = 0xFFFFFFFF, .u = 0,     .v = 0 },
            .{ .x =  cs, .y = -1.0, .z = -cs, .color = 0xFFFFFFFF, .u = 32767, .v = 0 },
            .{ .x =  cs, .y =  1.0, .z = -cs, .color = 0xFFFFFFFF, .u = 32767, .v = 32767 },
            .{ .x = -cs, .y =  1.0, .z = -cs, .color = 0xFFFFFFFF, .u = 0,     .v = 32767 },

            .{ .x = -cs, .y = -1.0, .z =  cs, .color = 0xFFFFFFFF, .u = 0,     .v = 0 },
            .{ .x =  cs, .y = -1.0, .z =  cs, .color = 0xFFFFFFFF, .u = 32767, .v = 0 },
            .{ .x =  cs, .y =  1.0, .z =  cs, .color = 0xFFFFFFFF, .u = 32767, .v = 32767 },
            .{ .x = -cs, .y =  1.0, .z =  cs, .color = 0xFFFFFFFF, .u = 0,     .v = 32767 },

            .{ .x = -cs, .y = -1.0, .z = -cs, .color = 0xFFFFFFFF, .u = 0,     .v = 0 },
            .{ .x = -cs, .y =  1.0, .z = -cs, .color = 0xFFFFFFFF, .u = 32767, .v = 0 },
            .{ .x = -cs, .y =  1.0, .z =  cs, .color = 0xFFFFFFFF, .u = 32767, .v = 32767 },
            .{ .x = -cs, .y = -1.0, .z =  cs, .color = 0xFFFFFFFF, .u = 0,     .v = 32767 },

            .{ .x =  cs, .y = -1.0, .z = -cs, .color = 0xFFFFFFFF, .u = 0,     .v = 0 },
            .{ .x =  cs, .y =  1.0, .z = -cs, .color = 0xFFFFFFFF, .u = 32767, .v = 0 },
            .{ .x =  cs, .y =  1.0, .z =  cs, .color = 0xFFFFFFFF, .u = 32767, .v = 32767 },
            .{ .x =  cs, .y = -1.0, .z =  cs, .color = 0xFFFFFFFF, .u = 0,     .v = 32767 },

            .{ .x = -cs, .y = -1.0, .z = -cs, .color = 0xFFFFFFFF, .u = 0,     .v = 0 },
            .{ .x = -cs, .y = -1.0, .z =  cs, .color = 0xFFFFFFFF, .u = 32767, .v = 0 },
            .{ .x =  cs, .y = -1.0, .z =  cs, .color = 0xFFFFFFFF, .u = 32767, .v = 32767 },
            .{ .x =  cs, .y = -1.0, .z = -cs, .color = 0xFFFFFFFF, .u = 0,     .v = 32767 },

            .{ .x = -cs, .y =  1.0, .z = -cs, .color = 0xFFFFFFFF, .u = 0,     .v = 0 },
            .{ .x = -cs, .y =  1.0, .z =  cs, .color = 0xFFFFFFFF, .u = 32767, .v = 0 },
            .{ .x =  cs, .y =  1.0, .z =  cs, .color = 0xFFFFFFFF, .u = 32767, .v = 32767 },
            .{ .x =  cs, .y =  1.0, .z = -cs, .color = 0xFFFFFFFF, .u = 0,     .v = 32767 },
        }),
        // zig fmt: on
    });

    // cube index buffer
    prim_ptr.index_buffer = sg.makeBuffer(.{
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

    prim_ptr.index_count = 36;

    return mesh;
}
