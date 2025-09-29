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
