package com.fer.NextMove.publisher;

import com.fer.NextMove.models.Trajectory;
import edu.wpi.rail.jrosbridge.Ros;
import edu.wpi.rail.jrosbridge.Topic;
import edu.wpi.rail.jrosbridge.messages.Message;
import edu.wpi.rail.jrosbridge.messages.geometry.Pose;
import org.ros.internal.message.RawMessage;

import javax.json.Json;
import javax.json.JsonObject;

public class CheckTrajectory extends Thread{

    Ros ros;
    Topic topic;
    private Trajectory trajectory;
    private int id;
    private Pose finishPose;
    private boolean exit = false;


    public CheckTrajectory(Trajectory trajectory, int id, Pose finishPose) {
        this.ros = new Ros("localhost");
        this.trajectory = trajectory;
        this.id = id;
        this.finishPose = finishPose;
        start();
    }

    public void run() {
        try {
            send();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void send() {
        while (!exit) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }finally {
                ros.connect();

                topic = new Topic(ros, "/plan_trajectory/check_trajectory", "std_msgs/String");
               /* std_msgs.String string = new std_msgs.String() {

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

                */

                String newString = id + "id";

                for(Pose p:trajectory.getTrajectoryPoses()) {
                    newString += p;
                }

                newString += finishPose;

                JsonObject call = Json.createObjectBuilder().
                        add("data", newString).build();

                Message message = new Message(call, "std_msgs/String");
                topic.publish(message);
                ros.disconnect();
            }
        }
    }

    public void disconnect(){
        ros.disconnect();
    }

    public void stopThread(){
        exit = true;
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

    public void setTrajectory(Trajectory trajectory) {
        this.trajectory = trajectory;
    }

    public int getTrajectoryId() {
        return id;
    }

    public void setTrajectoryId(int id) {
        this.id = id;
    }
}
