package com.jni.example;

public class TemperatureData {
    public String timestamp;
    public float temperature;
    public TemperatureScale scale;

    @Override
    public String toString() {
        return String.format("Timestamp = %s\nTemperature = %f\nScale = %s", timestamp, temperature, scale);
    }
}
