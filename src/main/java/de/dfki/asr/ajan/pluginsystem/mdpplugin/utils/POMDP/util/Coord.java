package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util;

import static java.lang.Math.abs;

public class Coord {
    public int x;
    public int y;

    public Coord(int _x, int _y) {
        x=_x;
        y=_y;
    }

    public static Coord Muliply(Coord c, int v){
        c.x *= v;
        c.y *= v;
        return c;
    }

    public static Coord Add(Coord first, Coord second){
        first.x += second.x;
        first.y += second.y;
        return first;
    }

    public static int ManhattanDistance(Coord c1, Coord c2) {
        return abs(c1.x - c2.x) + abs(c1.y - c2.y);
    };
}


;


