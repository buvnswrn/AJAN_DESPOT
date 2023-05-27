package com.jni.example;

import com.sun.jna.Native;
import com.sun.jna.Platform;

public class CMathDemo {

    public static void main(String[] args) {
        CMath INSTANCE = Native.load(Platform.isWindows() ? "msvcrt" : "c", CMath.class);
        double result = INSTANCE.cosh(100);
        System.out.println("Result:"+result);
    }
}
