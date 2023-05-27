package com.jni.example;


import com.ajan.POMDP.implementation.AJAN_Agent_State;

public class TemperatureSampler {
    static AJAN_Agent_State currentStateObject;
    static {
        System.loadLibrary("temperaturesampler");
    }

    public static void main(String[] args) {
        TemperatureSampler example = new TemperatureSampler();
        currentStateObject = new AJAN_Agent_State(-1,0.5);
        example.InitializeObject(currentStateObject);
        System.out.println("Sampled Temperature = "+example.getTemperature());
        TemperatureData temperatureData = example.getDetailedTemperature();
        if(temperatureData!= null){
            System.out.println(temperatureData);
        }
    }

    public TemperatureScale getPreferredScale() {
        return TemperatureScale.KELVIN;
    }

    public String getMessageString() {
        return "Message from Java";
    }

    public int setParams(int action, double reward, int obs, AJAN_Agent_State message) {
        System.out.println(message.scenario_id);
        System.out.println(message.state_id);
        System.out.println(message.weight);
        return (int) (1+action+reward+obs);
    }
    private native float getTemperature();
    private native TemperatureData getDetailedTemperature();
    private native void InitializeObject(AJAN_Agent_State State);
}