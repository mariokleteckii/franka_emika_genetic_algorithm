package com.fer.NextMove.models;

import edu.wpi.rail.jrosbridge.messages.geometry.Pose;

import java.util.ArrayList;
import java.util.List;

public class Population {

    private List<Trajectory> trajectories;
    private Pose startPose;
    private Pose finishPose;
    private int popSize;

    public Population() {
        trajectories = new ArrayList<>();
    }

    public Population(int popSize, Pose startPose, Pose finishPose) {
        this.startPose = startPose;
        this.finishPose = finishPose;
        this.popSize = popSize;
        trajectories = new ArrayList<>();
        createNewPopulation();
    }

    private void createNewPopulation(){
        for(int i = 0; i < popSize; i++){
            Trajectory trajectory = new Trajectory(startPose, finishPose);
            trajectory.calculateFitness(startPose, finishPose);
            trajectories.add(trajectory);
        }
    }


    public Trajectory getFittest(){
        Trajectory fittest = trajectories.get(0);

        for(int i = 1; i < trajectories.size(); i++){
            if(trajectories.get(i).getFitness() < fittest.getFitness()){
                fittest = trajectories.get(i);
            }
        }
        return fittest;
    }

    public boolean checkTrajectoryExistence(Trajectory trajectory){
        for(Trajectory t:trajectories){
            if(t.getFitness() == trajectory.getFitness()){
                return true;
            }
        }
        return false;
    }

    public Trajectory getTrajectory(int index){
        return trajectories.get(index);
    }


    public List<Trajectory> getTrajectories() {
        return trajectories;
    }

}
