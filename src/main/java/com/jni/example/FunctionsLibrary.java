package com.jni.example;

import com.sun.jna.Library;

public interface FunctionsLibrary extends Library {
    public int sum(int n1, int n2);
}
