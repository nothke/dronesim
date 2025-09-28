const std = @import("std");

const Entry = struct {
    key: []const u8,
    value: []const u8,
};

const whitespace = " \t";

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
            } else if (std.mem.indexOf(u8, lineTr, "=")) |index| {
                const keySlice = std.mem.trim(u8, lineTr[0..index], whitespace);
                const valueSlice = std.mem.trim(u8, lineTr[(index + 1)..], whitespace);

                // std.log.info("key: '{s}' value: '{s}'", .{ keySlice, valueSlice });
                return Entry{ .key = keySlice, .value = valueSlice };
            }
        } else |err| {
            switch (err) {
                error.EndOfStream => return null,
                else => return null,
            }
        }
    }
};

test "line_iterator" {
    var buff = [1024]u8{"firstLine=4\nsecondLine =    6"};
    var stream = std.io.fixedBufferStream(&buff);
    var ini = EntryReader{ .reader = stream.reader() };

    while (ini.next()) |res| {
        std.testing.expectEqual(res.key, "firstLine");
        std.testing.expectEqual(res.value, 4);
        return;
    }
}
