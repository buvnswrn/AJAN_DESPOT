package com.jni.example;

import com.sun.jna.Native;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;

public class FunctionsNative implements FunctionsLibrary{

    private final FunctionsLibrary functionsNative;
    public FunctionsNative(final String fileName) throws IOException{
        functionsNative = Native.loadLibrary(extractFile(fileName), FunctionsLibrary.class);
    }

    private String extractFile(final String fileName) throws IOException{
        final InputStream source = FunctionsNative.class.getClassLoader().getResourceAsStream(fileName);
        final File file = File.createTempFile("gcc/lib",null);
        assert source != null;
        FileUtils.copyInputStreamToFile(source,file);
        return file.getAbsolutePath();
    }

    @Override
    public int sum(int n1, int n2) {
        return functionsNative.sum(n1, n2);
    }
}
