package com.fer.NextMove;

import com.fer.NextMove.algorithm.GeneticAlgorithm;
import com.fer.NextMove.models.Trajectory;
import com.fer.NextMove.publisher.FinalTrajectory;
import com.fer.NextMove.subscriber.StartPose;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import edu.wpi.rail.jrosbridge.messages.geometry.Point;
import edu.wpi.rail.jrosbridge.messages.geometry.Pose;
import edu.wpi.rail.jrosbridge.messages.geometry.Quaternion;

@SpringBootApplication
public class NextMoveApplication {

	public static void main(String[] args) throws InterruptedException {
		SpringApplication.run(NextMoveApplication.class, args);

		Point point;
		Quaternion quaternion;
		Pose nextPosition;
		int populationSize;
		int maxGeneration;
		StartPose startPose = new StartPose();
		Pose startPosition = null;


		if (args.length == 7) {
			populationSize = 15;
			maxGeneration = 10;
		}
		else if(args.length == 8){
			populationSize = Integer.parseInt(args[7]);
			maxGeneration = 10;

			if(populationSize < 5){
				System.out.println("Minimalna velicina populacije je 5");
				System.exit(1);
			}
		}
		else if(args.length == 9){
			populationSize = Integer.parseInt(args[7]);
			maxGeneration = Integer.parseInt(args[8]);
		}
		else {
			throw new IllegalArgumentException("Minimalan broj argumenata je 7, a maksimalan 9. Prvih sedam argumenata " +
					"cine sljedecu poziciju, osmi argument je velicina populacije, a deveti argument je maksimalan broj " +
					"generacija za algoritam. Ako se ne unesu 8. i 9. argument oni su po defaultu 15 i 10");
		}


		point = new Point(Double.parseDouble(args[0]), Double.parseDouble(args[1]), Double.parseDouble(args[2]));
		quaternion = new Quaternion(Double.parseDouble(args[3]), Double.parseDouble(args[4]), Double.parseDouble(args[5]), Double.parseDouble(args[6]));

		nextPosition = new Pose(point, quaternion);


		while(startPosition == null) {
			Thread.sleep(50);
			startPosition = startPose.getPoseMsg();
		}

		startPose.unsubscribe();

		GeneticAlgorithm geneticAlgorithm = new GeneticAlgorithm(populationSize, startPosition, nextPosition, maxGeneration);
		Trajectory trajectory = geneticAlgorithm.callAlgorithm();
		trajectory.getTrajectoryPoses().add(nextPosition);

		System.out.println("Fitness: " + trajectory.getFitness());

		FinalTrajectory finalTrajectory = new FinalTrajectory(trajectory);
	}

}
