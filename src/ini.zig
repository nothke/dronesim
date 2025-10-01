const std = @import("std");

const Entry = struct {
    key: []const u8,
    value: []const u8,
};

const whitespace = " \t\r";

pub fn extractEntry(trimmed_line: []const u8) ?Entry {
    if (std.mem.indexOf(u8, trimmed_line, "=")) |index| {
        const keySlice = std.mem.trim(u8, trimmed_line[0..index], whitespace);
        const valueSlice = std.mem.trim(u8, trimmed_line[(index + 1)..], whitespace);

        // std.log.info("key: '{s}' value: '{s}'", .{ keySlice, valueSlice });
        return Entry{ .key = keySlice, .value = valueSlice };
    }

    return null;
}

pub const EntryReader = struct {
    reader: *std.io.Reader,

    pub fn next(iter: *EntryReader) ?Entry {
        while (iter.reader.takeDelimiterExclusive('\n')) |line| {
            //std.log.info("PROCESSING LINE: '{s}'", .{line});

            const lineTr = std.mem.trim(u8, line, whitespace);

            if (lineTr.len == 0)
                continue;

            // ignore comments
            if (lineTr[0] == '#' or lineTr[0] == ';') {
                {}
                // ignore sections (for now)
            } else if (lineTr[0] == '[') {
                {}
            } else if (extractEntry(lineTr)) |entry| {
                return entry;
            }
        } else |err| {
            switch (err) {
                error.EndOfStream => return null,
                else => return null,
            }
        }
    }
};

/// returns length of string
pub fn saveStruct(strc: anytype, writer: *std.io.Writer) !usize {
    const start = writer.end;
    inline for (std.meta.fields(@TypeOf(strc))) |field| {
        _ = try writer.write(field.name);
        _ = try writer.write(" = ");

        const info = @typeInfo(field.type);

        if (info == .int) {
            try writer.printIntAny(@field(strc, field.name), 10, .lower, .{});
        } else if (info == .float) {
            try writer.printFloat(@field(strc, field.name), .{});
        } else if (info == .pointer and info.pointer.size == .slice) {
            _ = try writer.write(@field(strc, field.name));
        } else {
            @compileError("Type not supported");
        }

        _ = try writer.write("\n");
    }

    return writer.end - start;
}

/// An allocator is used to store string values read from a file.
/// If you do not have any string values, you can set it to null.
/// The function doesn't return allocation pointers, so use an allocator that you can easily free in bulk like an arena.
pub fn loadStruct(strc: anytype, reader: *std.io.Reader, allocator: ?std.mem.Allocator) !void {
    var iter = EntryReader{ .reader = reader };

    if (@typeInfo(@TypeOf(strc)) != .pointer or @typeInfo(@TypeOf(strc.*)) != .@"struct")
        @compileError("Struct must be a pointer to a mutable struct");

    if (@typeInfo(@TypeOf(strc)).pointer.is_const)
        @compileError("Struct must be mutable");

    while (iter.next()) |entry| {
        inline for (std.meta.fields(@TypeOf(strc.*))) |field| {
            if (std.mem.eql(u8, field.name, entry.key)) {
                const info = @typeInfo(field.type);

                if (info == .int) {
                    @field(strc.*, field.name) = try std.fmt.parseInt(field.type, entry.value, 10);
                } else if (info == .float) {
                    @field(strc.*, field.name) = try std.fmt.parseFloat(field.type, entry.value);
                } else if (info == .pointer and info.pointer.size == .slice) {
                    if (allocator) |alloc| {
                        @field(strc.*, field.name) = try alloc.dupeZ(u8, entry.value);
                    } else {
                        return error.AllocatorCantBeNull;
                    }
                    //@field(strc, field.name)
                } else {
                    @compileError("Type not supported");
                }
            }
        }
    }
}

test "read_struct" {
    const string =
        \\firstLine=4
        \\secondLine   =  6
        \\   [section]
        \\# comment = even if it has a = sign
        \\nothing = at all
        \\
        \\
    .*;

    var strc = struct {
        firstLine: i32 = 8,
        secondLine: f32 = 33.4,
        nothing: []const u8 = "",
    }{};

    var reader = std.io.Reader.fixed(&string);

    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    try loadStruct(&strc, &reader, arena.allocator());
    defer arena.deinit();

    try std.testing.expectEqual(strc.firstLine, 4);
    try std.testing.expectEqual(strc.secondLine, 6);
    try std.testing.expectEqualStrings(strc.nothing, "at all");
}

test "write_struct" {
    var buff = std.mem.zeroes([1024]u8);
    var writer = std.io.Writer.fixed(&buff);

    const strc = struct {
        num: i32 = 8,
        flt: f32 = 33.4,
        str: []const u8 = "something",
    }{};

    const len = try saveStruct(strc, &writer);

    try std.testing.expectEqualStrings("num = 8\nflt = 33.4\nstr = something\n", buff[0..len]);
}

test "line_iterator" {
    const string =
        \\firstLine=4
        \\secondLine   =  6
        \\   [section]
        \\# comment = even if it has a = sign
        \\nothing = at all
        \\
        \\
    .*;

    var reader = std.io.Reader.fixed(&string);
    var ini = EntryReader{ .reader = &reader };

    const eql = std.testing.expectEqualStrings;

    var i: i32 = 0;
    while (ini.next()) |res| : (i += 1) {
        switch (i) {
            0 => {
                try eql(res.key, "firstLine");
                try eql(res.value, "4");
            },
            1 => {
                try eql(res.key, "secondLine");
                try eql(res.value, "6");
            },
            2 => {
                try eql(res.key, "nothing");
                try eql(res.value, "at all");
            },
            else => {},
        }
    }
}
