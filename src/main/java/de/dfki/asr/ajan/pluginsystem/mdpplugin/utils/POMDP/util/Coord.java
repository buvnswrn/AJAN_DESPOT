package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.util;

public class Coord {
    public int x;
    public int y;

    Coord(int _x, int _y) {
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

}


;


