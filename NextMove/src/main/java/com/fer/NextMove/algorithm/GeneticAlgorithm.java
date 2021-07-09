package com.fer.NextMove.algorithm;

import com.fer.NextMove.messages.TrajectoryResultInfo;
import com.fer.NextMove.models.Population;
import com.fer.NextMove.models.Trajectory;
import com.fer.NextMove.publisher.CheckTrajectory;
import com.fer.NextMove.subscriber.TrajectoryResult;
import edu.wpi.rail.jrosbridge.messages.geometry.Point;
import edu.wpi.rail.jrosbridge.messages.geometry.Pose;
import edu.wpi.rail.jrosbridge.messages.geometry.Quaternion;

import java.util.ArrayList;
import java.util.List;

public class GeneticAlgorithm {

    private static final double uniformRate = 0.5;
    private static final double mutationRate = 0.025;
    private static final double mutationStep = 0.055;
    private static final int tournamentSize = 6;
    private static final double minFraction = 100;

    private Pose startPose;
    private Pose finishPose;
    private int maxGeneration;
    private int populationSize;

    private int id = 1;
    int generation = 1;

    public GeneticAlgorithm(int populationSize, Pose startPose, Pose finishPose, int maxGeneration) {
        this.populationSize = populationSize;
        this.startPose = startPose;
        this.finishPose = finishPose;
        this.maxGeneration = maxGeneration + 1;
    }

    public Trajectory callAlgorithm() throws InterruptedException {

        Population population = new Population(populationSize, startPose, finishPose);

        double maxFitness = getMaxFitness();

        System.out.println("startPose: " + startPose.getPosition().getX() + "\n" +
                startPose.getPosition().getY() + "\n" + startPose.getPosition().getZ());
        System.out.println("maxFitness: " + maxFitness);
        System.out.println("Fittest's fitness: " + population.getFittest().getFitness());



        while((population.getFittest().getFitness() > maxFitness && generation < maxGeneration) || id == 1) {
            System.out.println("Fittest's fitness: " + population.getFittest().getFitness());
            System.out.println("maxFitness: " + maxFitness);
            population = evolve(population);
            generation++;
        }

        System.out.println("Best pose: ");
        for(Pose p:population.getFittest().getTrajectoryPoses()) {
            System.out.println(p);
        }

        return population.getFittest();
    }


    private Population evolve(Population oldPopulation) throws InterruptedException {
        Population newPopulation = new Population();

        TrajectoryResultInfo trajectoryResultInfo;

        int k;

        if(id == 1 && checkFraction(oldPopulation.getFittest()) < minFraction){
            k = 0;
        }
        else {
            k = 1;
            newPopulation.getTrajectories().add(oldPopulation.getFittest());
        }

        for(int i = k; i < oldPopulation.getTrajectories().size(); i++) {

            boolean checkTrExistence = true;
            Trajectory newTrajectory = new Trajectory();

            while(checkTrExistence) {
                Trajectory trajectory1 = getBestRandomTrajectory(oldPopulation);
                Trajectory trajectory2 = getBestRandomTrajectory(oldPopulation);

                while (trajectory2.getFitness() == trajectory1.getFitness()) {
                    trajectory2 = getBestRandomTrajectory(oldPopulation);
                }

                newTrajectory = crossover(trajectory1, trajectory2);
                mutate(newTrajectory);

                newTrajectory.calculateFitness(startPose, finishPose);

                System.out.println("New trajectory fittness: " + newTrajectory.getFitness());

                checkTrExistence = newPopulation.checkTrajectoryExistence(newTrajectory);
            }

            CheckTrajectory checkTrajectory = new CheckTrajectory(newTrajectory, id, finishPose);
            TrajectoryResult trajectoryResult = new TrajectoryResult();
            trajectoryResultInfo = trajectoryResult.getTrajectoryResultInfo();

            while (trajectoryResultInfo == null) {
                try {
                    Thread.sleep(50);
                    trajectoryResultInfo = trajectoryResult.getTrajectoryResultInfo();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            while (Integer.valueOf(trajectoryResultInfo.getData().getId()) != id) {
                trajectoryResultInfo = trajectoryResult.getTrajectoryResultInfo();
            }

            checkTrajectory.stopThread();
            trajectoryResult.unsubscribe();
            id++;

            double fraction = Double.valueOf(trajectoryResultInfo.getData().getFraction());

            if(fraction < minFraction){
                i--;
            }
            else{
                newPopulation.getTrajectories().add(newTrajectory);
            }

            System.out.println("Velicina populacije: " + newPopulation.getTrajectories().size());
            System.out.println("Generation: " + generation);
            System.out.println("Fraction: " + fraction);
            Thread.sleep(1000);
        }

        return newPopulation;
    }

    private double checkFraction(Trajectory newTrajectory){

        CheckTrajectory checkTrajectory = new CheckTrajectory(newTrajectory, id, finishPose);
        TrajectoryResult trajectoryResult = new TrajectoryResult();
        TrajectoryResultInfo trajectoryResultInfo = trajectoryResult.getTrajectoryResultInfo();

        while (trajectoryResultInfo == null) {
            try {
                Thread.sleep(50);
                trajectoryResultInfo = trajectoryResult.getTrajectoryResultInfo();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        while (Integer.valueOf(trajectoryResultInfo.getData().getId()) != id) {
            trajectoryResultInfo = trajectoryResult.getTrajectoryResultInfo();
        }

        checkTrajectory.stopThread();
        trajectoryResult.unsubscribe();
        id++;

        double fraction = Double.valueOf(trajectoryResultInfo.getData().getFraction());
        System.out.println(fraction);

        return fraction;
    }

    private Trajectory crossover(Trajectory trajectory1, Trajectory trajectory2){
        Trajectory trajectory = new Trajectory();
        List<Double> valuesTr1 = new ArrayList<>();
        List<Double> valuesTr2 = new ArrayList<>();
        List<Double> newValues = new ArrayList<>();
        ArrayList<Pose> trajectoryList = new ArrayList<>();
        Pose pose;

        for(int k = 0; k < trajectory1.getTrajectoryPoses().size(); k++){
            Pose pose1 = trajectory1.getTrajectoryPoses().get(k);

            valuesTr1.add(pose1.getPosition().getX());
            valuesTr1.add(pose1.getPosition().getY());
            valuesTr1.add(pose1.getPosition().getZ());
            valuesTr1.add(pose1.getOrientation().getX());
            valuesTr1.add(pose1.getOrientation().getY());
            valuesTr1.add(pose1.getOrientation().getZ());
            valuesTr1.add(pose1.getOrientation().getW());

            Pose pose2 = trajectory2.getTrajectoryPoses().get(k);

            valuesTr2.add(pose2.getPosition().getX());
            valuesTr2.add(pose2.getPosition().getY());
            valuesTr2.add(pose2.getPosition().getZ());
            valuesTr2.add(pose2.getOrientation().getX());
            valuesTr2.add(pose2.getOrientation().getY());
            valuesTr2.add(pose2.getOrientation().getZ());
            valuesTr2.add(pose2.getOrientation().getW());

        }

        for(int i = 0; i < valuesTr1.size(); i++){
            if(Math.random() < uniformRate){
                newValues.add(valuesTr1.get(i));
            }
            else {
                newValues.add(valuesTr2.get(i));
            }
        }

        for(int j = 0; j < newValues.size(); j+=7){
            pose = new Pose(new Point(newValues.get(j), newValues.get(j+1), newValues.get(j+2)),
                            new Quaternion(newValues.get(j+3), newValues.get(j+4), newValues.get(j+5), newValues.get(j+6)));
            trajectoryList.add(pose);
        }

        trajectory.setTrajectoryList(trajectoryList);

        return trajectory;
    }

    private void mutate(Trajectory trajectory){
        ArrayList<Pose> poses = trajectory.getTrajectoryPoses();
        List<Double> coordinates = new ArrayList<>();
        Point point;
        double x;
        double y;
        double z;
        int index;

        for(int i = 0; i < poses.size(); i++) {
            coordinates.add(poses.get(i).getPosition().getX());
            coordinates.add(poses.get(i).getPosition().getY());
            coordinates.add(poses.get(i).getPosition().getZ());

            index = (int)(Math.random()*(coordinates.size()-1));

            if (Math.random() < mutationRate) {
                coordinates.set(index, coordinates.get(index) + mutationStep);
            }
            x = coordinates.get(0);
            y = coordinates.get(1);
            z = coordinates.get(2);

            point = new Point(x, y, z);
            Pose pose = new Pose(point, trajectory.getTrajectoryPoses().get(i).getOrientation());
            trajectory.getTrajectoryPoses().set(i, pose);
            coordinates.clear();
        }
    }

    private Trajectory getBestRandomTrajectory(Population population){
        Population tournamentPopulation = new Population();
        int index;

        for(int i = 0; i < tournamentSize; i++){
            index = (int) (Math.random() * population.getTrajectories().size());
            tournamentPopulation.getTrajectories().add(population.getTrajectory(index));
        }

        return tournamentPopulation.getFittest();
    }

    private double getMaxFitness(){
        double xS = startPose.getPosition().getX();
        double yS = startPose.getPosition().getY();
        double zS = startPose.getPosition().getZ();
        double xN = finishPose.getPosition().getX();
        double yN = finishPose.getPosition().getY();
        double zN = finishPose.getPosition().getZ();

        return 1.5 * Math.sqrt((xN - xS) * (xN - xS) + (yN - yS) * (yN - yS) + (zN - zS) * (zN - zS));

    }
}
