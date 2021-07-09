package com.fer.NextMove.subscriber;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fer.NextMove.messages.TrajectoryResultInfo;
import edu.wpi.rail.jrosbridge.Ros;
import edu.wpi.rail.jrosbridge.Topic;
import edu.wpi.rail.jrosbridge.callback.TopicCallback;
import edu.wpi.rail.jrosbridge.messages.Message;

import java.io.IOException;

public class TrajectoryResult {

    Topic topic;
    private Ros ros;
    private ObjectMapper objectMapper = new ObjectMapper();
    private String result;
    private TrajectoryResultInfo trajectoryResultInfo;


    public TrajectoryResult() {
        this.ros = new Ros("localhost");
        subscribe();
    }

    public void subscribe() {

        ros.connect();
        topic = new Topic(ros, "/plan_trajectory/trajectory_result", "std_msgs/String");

        topic.subscribe(new TopicCallback() {

            @Override
            public void handleMessage(Message message) {

                try {
                    result = message.toString();
                    result = result.replace("\\", "");
                    result = result.replace(":\"", ":");
                    result = result.replace("}\"", "}");

                    trajectoryResultInfo = objectMapper.readValue(result, TrajectoryResultInfo.class);

                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });
    }

    public TrajectoryResultInfo getTrajectoryResultInfo() {
        return trajectoryResultInfo;
    }


    public void unsubscribe(){
        topic.unsubscribe();
        ros.disconnect();
    }
}
