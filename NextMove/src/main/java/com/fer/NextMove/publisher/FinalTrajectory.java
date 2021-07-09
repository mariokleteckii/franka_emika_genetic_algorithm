package com.fer.NextMove.publisher;


import com.fer.NextMove.models.Trajectory;
import edu.wpi.rail.jrosbridge.Ros;
import edu.wpi.rail.jrosbridge.Topic;
import edu.wpi.rail.jrosbridge.messages.Message;
import org.ros.internal.message.RawMessage;

import javax.json.Json;
import javax.json.JsonObject;


public class FinalTrajectory extends Thread{

    Ros ros;
    Topic topic;
    private Trajectory trajectory;

    public FinalTrajectory(Trajectory trajectory) {
        this.ros = new Ros("localhost");
        this.trajectory = trajectory;
        start();
    }

    public void run() {
        try {
            send(trajectory);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void send(Trajectory trajectory) {
        while (true) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }finally {
                ros.connect();

                topic = new Topic(ros, "/plan_trajectory/final_trajectory", "std_msgs/String");
                std_msgs.String string = new std_msgs.String() {

                    String st = "";

                    @Override
                    public java.lang.String getData() {
                        return st;
                    }

                    @Override
                    public void setData(java.lang.String s) {
                        st += s;
                    }

                    @Override
                    public RawMessage toRawMessage() {
                        return null;
                    }
                };

                string.setData(trajectory.getTrajectoryPoses().toString());
                JsonObject call = Json.createObjectBuilder().add("data", string.getData()).build();

                Message message = new Message(call, "std_msgs/String");

                topic.publish(message);
                ros.disconnect();
            }
        }
    }

    public void disconnect(){
        ros.disconnect();
    }
}
