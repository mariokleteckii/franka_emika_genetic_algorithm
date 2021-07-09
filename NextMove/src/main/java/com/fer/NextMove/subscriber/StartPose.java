package com.fer.NextMove.subscriber;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.rail.jrosbridge.Ros;
import edu.wpi.rail.jrosbridge.Topic;
import edu.wpi.rail.jrosbridge.callback.TopicCallback;
import edu.wpi.rail.jrosbridge.messages.Message;
import edu.wpi.rail.jrosbridge.messages.geometry.Pose;

import java.io.IOException;

public class StartPose {

    Topic topic;
    private Ros ros;
    private ObjectMapper objectMapper = new ObjectMapper();
    private Pose poseMsg;


    public StartPose() {
        this.ros = new Ros("localhost");
        subscribe();
    }

    public void subscribe() {

        ros.connect();
        topic = new Topic(ros, "/plan_trajectory/start_pose", "geometry_msgs/Pose");

        topic.subscribe(new TopicCallback() {
            @Override
            public void handleMessage(Message message) {

                try {
                    poseMsg = objectMapper.readValue(message.toJsonObject().toString(), Pose.class);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });
    }

    public Pose getPoseMsg() {
        return poseMsg;
    }

    public void unsubscribe(){
        topic.unsubscribe();
        ros.disconnect();
    }
}
