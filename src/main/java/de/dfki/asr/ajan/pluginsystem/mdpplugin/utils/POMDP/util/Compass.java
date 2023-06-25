package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util;

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
}
