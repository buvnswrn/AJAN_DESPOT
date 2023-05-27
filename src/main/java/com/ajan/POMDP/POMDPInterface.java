package com.ajan.POMDP;

public abstract class POMDPInterface {
    static {
        System.loadLibrary("pomdp");
    }

    private native boolean Step();
}
