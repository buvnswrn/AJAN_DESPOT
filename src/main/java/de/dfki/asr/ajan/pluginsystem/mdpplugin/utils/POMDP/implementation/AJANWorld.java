package de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.implementation;

import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.State;
import de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.World;
import org.apache.http.client.ResponseHandler;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.impl.client.BasicResponseHandler;
import org.apache.http.impl.client.CloseableHttpClient;
import org.apache.http.impl.client.HttpClients;
import org.json.JSONArray;
import org.json.JSONObject;

import java.io.IOException;
import java.util.HashMap;

public class AJANWorld extends World {
    HashMap<String,Object> variables;
    public AJAN_Agent ajanAgent;

    public AJANWorld(AJAN_Agent ajanAgent) {
        this.ajanAgent = ajanAgent;
        variables = new HashMap<>();
        variables.put("ONE",1);
        variables.put("N_BEAMS",1);
        variables.put("BITS_PER_READING",1);
        variables.put("TAG",4);
    }

    @Override
    public boolean Connect(){
        try {
            InitializeROSClient();
        } catch (IOException e) {
            System.err.println("Error in Connecting to ROS Client"+e.getMessage());
        }
        return false;
    }

    @Override
    public State Initialize() {
        return null;
    }

    @Override
    public State GetCurrentState() {
        return null;
    }

    @Override
    public void PrintState(State state) {

    }

    @Override
    public boolean ExecuteAction(int action, int obs) {
        try {
            String executeActionResponse = ExecuteAction(action);
            JSONObject jsonObject = new JSONObject(executeActionResponse);
            JSONArray observations = (JSONArray) jsonObject.get("observations");
            for (int dir = 0; dir < observations.length(); dir++) {
                SetReading((int) observations.get(dir), dir);
                // OPTIMIZE: Check for converting the data type to long instead of int
            }
        } catch (IOException e) {
            System.err.println("Error in Executing Action:"+e.getMessage());
        }
        return false;
    }
    private static String ExecuteAction(int action) throws IOException {
        String URI = "http://127.0.0.1:8000/AJAN/ros/call-service/?service_name=laser_tag_action_obs&action="+ action;
        return postRequest(URI);
    }

    private static String postRequest(String URI) throws IOException {
        ResponseHandler<String> responseHandler= new BasicResponseHandler();
        CloseableHttpClient httpClient = HttpClients.createDefault();
        HttpPost httpPost = new HttpPost(URI);
        return httpClient.execute(httpPost, responseHandler);
    }

    private static String InitializeROSClient() throws IOException {
        String URI = "http://127.0.0.1:8000/AJAN/ros/initialize-service-client/?service_name=laser_tag_action_obs";
        return postRequest(URI);
    }
    private void SetReading(int observation, int dir) {
        AJAN_Agent.currentObservation &=  ~(((int)variables.get("ONE")<< (int)variables.get("BITS_PER_READING")-1)
                                        <<(dir *(int)variables.get("BITS_PER_READING")));
        AJAN_Agent.currentObservation |= observation << (dir * (int)variables.get("BITS_PER_READING"));
    }
    public int getCurrentObservation(){
        return AJAN_Agent.currentObservation;
    }


}
