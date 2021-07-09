package com.fer.NextMove.models;

import edu.wpi.rail.jrosbridge.messages.geometry.Point;
import edu.wpi.rail.jrosbridge.messages.geometry.Pose;
import edu.wpi.rail.jrosbridge.messages.geometry.Quaternion;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Trajectory {

    double fitness;
    Pose startPose;
    Pose finishPose;
    ArrayList<Pose> trajectoryList;

    public Trajectory(){
        trajectoryList = new ArrayList<>();
    }

    public Trajectory(Pose startPose, Pose finishPose) {

        this.startPose = startPose;
        this.finishPose = finishPose;
        trajectoryList = new ArrayList<>();
        createNewPoses(startPose, finishPose);
        calculateFitness(startPose, finishPose);
    }

    private void createNewPoses(Pose sPose, Pose fPose){

        Point sPosition = sPose.getPosition();
        Quaternion sOrientation = sPose.getOrientation();

        Point fPosition = fPose.getPosition();
        Quaternion fOrientation = fPose.getOrientation();

        List<Double> initialValues = new ArrayList<>();
        List<Double> finishValues = new ArrayList<>();
        List<Double> newValues = new ArrayList<>();


        initialValues.add(sPosition.getX());
        initialValues.add(sPosition.getY());
        initialValues.add(sPosition.getZ());
        initialValues.add(sOrientation.getX());
        initialValues.add(sOrientation.getY());
        initialValues.add(sOrientation.getZ());
        initialValues.add(sOrientation.getW());


        finishValues.add(fPosition.getX());
        finishValues.add(fPosition.getY());
        finishValues.add(fPosition.getZ());
        finishValues.add(fOrientation.getX());
        finishValues.add(fOrientation.getY());
        finishValues.add(fOrientation.getZ());
        finishValues.add(fOrientation.getW());

        double min;
        double max;
        double value;
        Pose pose;

        for(int j = 0; j < 7; j++){
            for(int i = 0; i < finishValues.size(); i++){

                min = initialValues.get(i);
                max = finishValues.get(i);
                value = (Math.random() * (max - min)) + min;

                if(j==0) {
                    newValues.add(value);
                }
                else{
                    newValues.add(i, value);
                }
            }
            pose = new Pose(new Point(newValues.get(0), newValues.get(1), newValues.get(2)),
                    new Quaternion(finishValues.get(3), finishValues.get(4), finishValues.get(5), finishValues.get(6)));

            trajectoryList.add(pose);
        }

        /*
        for(int i = 0; i < finishValues.size(); i++){

            min = initialValues.get(i);
            max = finishValues.get(i);
            value = (Math.random() * (max - min)) + min;

            newValues.add(value);
        }
        pose = new Pose(new Point(newValues.get(0), newValues.get(1), newValues.get(2)),
                            new Quaternion(finishValues.get(3), finishValues.get(4), finishValues.get(5), finishValues.get(6)));


     //   pose = new Pose(new Point(newValues.get(0), newValues.get(1), newValues.get(2)),
       //         new Quaternion(newValues.get(3), newValues.get(4), newValues.get(5), newValues.get(6)));
        trajectoryList.add(pose);

        for(int i = 0; i < initialValues.size(); i++){

            min = initialValues.get(i);
            max = finishValues.get(i);
            value = (Math.random() * (max - min)) + min;    

            newValues.add(i, value);
        }

       // pose = new Pose(new Point(newValues.get(0), newValues.get(1), newValues.get(2)),
         //       new Quaternion(newValues.get(3), newValues.get(4), newValues.get(5), newValues.get(6)));
        pose = new Pose(new Point(newValues.get(0), newValues.get(1), newValues.get(2)),
                new Quaternion(finishValues.get(3), finishValues.get(4), finishValues.get(5), finishValues.get(6)));

        trajectoryList.add(pose);

        for(int i = 0; i < initialValues.size(); i++){

            min = initialValues.get(i);
            max = finishValues.get(i);
            value = (Math.random() * (max - min)) + min;

            newValues.add(i, value);
        }
        pose = new Pose(new Point(newValues.get(0), newValues.get(1), newValues.get(2)),
                new Quaternion(finishValues.get(3), finishValues.get(4), finishValues.get(5), finishValues.get(6)));

       // pose = new Pose(new Point(newValues.get(0), newValues.get(1), newValues.get(2)),
         //       new Quaternion(newValues.get(3), newValues.get(4), newValues.get(5), newValues.get(6)));
        trajectoryList.add(pose);

        for(int i = 0; i < initialValues.size(); i++){

            min = initialValues.get(i);
            max = finishValues.get(i);
            value = (Math.random() * (max - min)) + min;

            newValues.add(i, value);
        }
        pose = new Pose(new Point(newValues.get(0), newValues.get(1), newValues.get(2)),
                new Quaternion(finishValues.get(3), finishValues.get(4), finishValues.get(5), finishValues.get(6)));


        // pose = new Pose(new Point(newValues.get(0), newValues.get(1), newValues.get(2)),
         //       new Quaternion(newValues.get(3), newValues.get(4), newValues.get(5), newValues.get(6)));
        trajectoryList.add(pose);
        */
     //   trajectoryList.add(pose1);
      //  Collections.addAll(trajectoryList, pose1, pose2, pose3, pose4);

    }

    public ArrayList<Pose> getTrajectoryPoses() {
        return trajectoryList;
    }

    public double getFitness() {
        return fitness;
    }

    public void calculateFitness(Pose startPose, Pose finishPose) {

        fitness = getDistanceBetweenTwoPoses(startPose, trajectoryList.get(0));

        for(int i = 0; i < (trajectoryList.size() - 1); i++){
            fitness += getDistanceBetweenTwoPoses(trajectoryList.get(i), trajectoryList.get(i+1));
        }

        fitness += getDistanceBetweenTwoPoses(trajectoryList.get(trajectoryList.size()-1), finishPose);

    }

    public double getDistanceBetweenTwoPoses(Pose pose1, Pose pose2){
        double xS = pose1.getPosition().getX();
        double yS = pose1.getPosition().getY();
        double zS = pose1.getPosition().getZ();
        double xN = pose2.getPosition().getX();
        double yN = pose2.getPosition().getY();
        double zN = pose2.getPosition().getZ();

        return Math.sqrt((xN - xS) * (xN - xS) + (yN - yS) * (yN - yS) + (zN - zS) * (zN - zS));
    }


    public void setTrajectoryList(ArrayList<Pose> trajectoryList) {
        this.trajectoryList = trajectoryList;
    }
}
