package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util;

import static de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util.Coord.Add;

public enum Compass {
    NORTH, EAST, SOUTH, WEST, NORTHEAST, SOUTHEAST, SOUTHWEST, NORTHWEST;

    public static Coord DIRECTIONS[] = {new Coord(0, 1),
            new Coord(1, 0),
            new Coord(0, -1),
            new Coord(-1, 0),
            new Coord(1, 1),
            new Coord(1, -1),
            new Coord(-1, -1),
            new Coord(-1, 1)};

    public static int Opposite(int dir) {
        return (dir + 2) % 4;
    }

    public static boolean Opposite(int dir1, int dir2) {
        return Add(DIRECTIONS[dir1], DIRECTIONS[dir2]).equals(new Coord(0, 0));
    }
}
