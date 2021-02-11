// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.DriveConstants;

/**
 * Class that defines and caches all trajectories that the robot could run.
 * Create one object instance of this class when the robot initializes to build the trajectories. 
 */
public class TrajectoryCache {
    private FileLog log;
   
    private static int trajectoryCount = 7;
    public Trajectory[] cache = new Trajectory[trajectoryCount];

    public enum TrajectoryType {
        test(0),
        testCurve(1),
        opponentTrenchPickup(2),
        bounceSToA3(3),
        bounceA3ToA6(4),
        bounceA6ToA9(5),
        bounceA9ToF(6);
        // bounceA3ToA6p1(4),
        // bounceA3ToA6p2(5),
        // bounceA3ToA6p3(6),
        // bounceA6ToA9p1(7),
        // bounceA6ToA9p2(8),
        // bounceA6ToA9p3(8),
        // bounceA6ToA9p4(9),
        // bounceA6ToA9p5(10),

    
        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        TrajectoryType(int value) { this.value = value; }
    }
	
    /**
     * Build all trajectories in this.cache[] for trajectory-following commands.
     * @param log
     */
    public TrajectoryCache(FileLog log){
        this.log = log;
        cache[TrajectoryType.test.value] = calcTrajectory("Test", 0.4, 0.4, false, 
            new Pose2d(0, 0, new Rotation2d(0.0)),
            List.of(),
            new Pose2d(6.0, 0, new Rotation2d(Math.toRadians(0.0)))
        );

        cache[TrajectoryType.testCurve.value] = calcTrajectory("Test Curve", 0.4, 0.4, false, 
            new Pose2d(0, 0, new Rotation2d(0.0)),
            List.of(),
            new Pose2d(3, 3, new Rotation2d(Math.toRadians(90.0)))
        );

        // drive from line to trench (assumes starting directly in front and facing target)		  
        // the trajectory to follow (all units in meters)
        // start at the origin facing the +X direction
        // Y is distance forward
        // firstBallY = -1.4
        // firstBallX = -3.1
        // distance between first and last ball = 1.82
        cache[TrajectoryType.opponentTrenchPickup.value] = calcTrajectory("Opponent Trench", 0.4, 0.4, true, 
            new Pose2d(0, 0, new Rotation2d(0.0)),
            List.of(),
            new Pose2d(-3, 3, new Rotation2d(Math.toRadians(90.0)))
        );

        cache[TrajectoryType.bounceSToA3.value] = calcTrajectory("Bounce S-A3", 0.3, 0.4, false, 
            new Pose2d(0.762, 2.286, new Rotation2d(0.0)),
            List.of(),
            new Pose2d(2.286, 3.81, new Rotation2d(Math.toRadians(90.0)))
        );

        // TODO Remove when actual bounce path is finalized (uncomment for "perfect" bounce path)
        // cache[TrajectoryType.bounceA3ToA6p1.value] = calcTrajectory("Bounce A3-6 Part 1", 0.3, 0.4, true, 
        //     new Pose2d(2.286, 3.81, new Rotation2d(Math.toRadians(90.0))),
        //     List.of(),
        //     new Pose2d(3.048, 1.524, new Rotation2d(Math.toRadians(90.0)))
        // );

        // cache[TrajectoryType.bounceA3ToA6p2.value] = calcTrajectory("Bounce A3-6 Part 2", 0.3, 0.4, true, 
        //     new Pose2d(3.048, 1.524, new Rotation2d(Math.toRadians(90.0))),
        //     List.of(new Translation2d(3.81, 0.762)),
        //     new Pose2d(4.572, 1.524, new Rotation2d(Math.toRadians(-90.0)))
        // );

        // cache[TrajectoryType.bounceA3ToA6p2.value] = calcTrajectory("Bounce A3-6 Part 3", 0.3, 0.4, true, 
        //     new Pose2d(4.572, 1.524, new Rotation2d(Math.toRadians(-90.0))),
        //     List.of(),
        //     new Pose2d(4.572, 3.81, new Rotation2d(Math.toRadians(-90.0)))
        // );

        // cache[TrajectoryType.bounceA6ToA9p1.value] = calcTrajectory("Bounce A6-9 Part 1", 0.3, 0.4, false, 
        //     new Pose2d(4.572, 3.81, new Rotation2d(Math.toRadians(-90.0))),
        //     List.of(),
        //     new Pose2d(4.572, 1.524, new Rotation2d(Math.toRadians(-90.0)))
        // );

        // cache[TrajectoryType.bounceA6ToA9p2.value] = calcTrajectory("Bounce A6-9 Part 2", 0.3, 0.4, false, 
        //     new Pose2d(4.572, 1.524, new Rotation2d(Math.toRadians(-90.0))),
        //     List.of(),
        //     new Pose2d(5.334, 0.762, new Rotation2d(Math.toRadians(0.0)))
        // );

        // cache[TrajectoryType.bounceA6ToA9p3.value] = calcTrajectory("Bounce A6-9 Part 2", 0.3, 0.4, false, 
        //     new Pose2d(5.334, 0.762, new Rotation2d(Math.toRadians(0.0))),
        //     List.of(),
        //     new Pose2d(6.096, 0.762, new Rotation2d(Math.toRadians(0.0)))
        // );

        // cache[TrajectoryType.bounceA6ToA9p4.value] = calcTrajectory("Bounce A6-9 Part 3", 0.3, 0.4, false, 
        //     new Pose2d(6.096, 0.762, new Rotation2d(Math.toRadians(0.0))),
        //     List.of(),
        //     new Pose2d(6.858, 1.524, new Rotation2d(Math.toRadians(90.0)))
        // );

        // cache[TrajectoryType.bounceA6ToA9p5.value] = calcTrajectory("Bounce A6-9 Part 3", 0.3, 0.4, false, 
        //     new Pose2d(6.858, 1.524, new Rotation2d(Math.toRadians(90.0))),
        //     List.of(),
        //     new Pose2d(6.858, 3.81, new Rotation2d(Math.toRadians(90.0)))
        // );

        cache[TrajectoryType.bounceA3ToA6.value] = calcTrajectory("Bounce A3-6", 0.3, 0.4, true, 
            new Pose2d(2.286, 3.81, new Rotation2d(Math.toRadians(90.0))),
            // List.of(new Translation2d(2.5, 3),
            //         new Translation2d(2.9777, 2.01),
            //         new Translation2d(3.015, 1.859),
            //         new Translation2d(3.048, 1.524),
            //         new Translation2d(3.27118, 0.98518),
            //         new Translation2d(3.81, 0.762),
            //         new Translation2d(4.34882, 0.98518),
            //         new Translation2d(4.572, 1.524),
            //         new Translation2d(4.572, 1.6),
            //         new Translation2d(4.572, 1.9),
            //         new Translation2d(4.572, 3)),
            List.of(new Translation2d(3, 1.5),
            new Translation2d(4, 1),
            new Translation2d(4.572, 1.524)),
            new Pose2d(4.572, 3.81, new Rotation2d(Math.toRadians(-90.0)))
        );

        cache[TrajectoryType.bounceA6ToA9.value] = calcTrajectory("Bounce A6-9", 0.3, 0.4, false, 
            new Pose2d(4.572, 3.81, new Rotation2d(Math.toRadians(-90.0))),
            // List.of(new Translation2d(4.572, 3),
            //         new Translation2d(4.572, 1.9),
            //         new Translation2d(4.572, 1.6),
            //         new Translation2d(4.572, 1.524),
            //         new Translation2d(4.67409, 1.143),
            //         new Translation2d(4.79518, 0.98518),
            //         new Translation2d(4.953, 0.86409),
            //         new Translation2d(5.334, 0.762),
            //         new Translation2d(5.5, 0.762),
            //         new Translation2d(5.8, 0.762),
            //         new Translation2d(6.096, 0.762),
            //         new Translation2d(6.477, 0.864089),
            //         new Translation2d(6.634815, 0.98518),
            //         new Translation2d(6.755911, 1.143),
            //         new Translation2d(6.858, 1.524),
            //         new Translation2d(6.858, 1.6),
            //         new Translation2d(6.858, 1.9),
            //         new Translation2d(6.858, 3)),
            List.of(new Translation2d(4.572, 1.524),
            new Translation2d(5.715, 1),
            new Translation2d(6.858, 1.524)),
            new Pose2d(6.858, 3.81, new Rotation2d(Math.toRadians(90.0)))
        );

        cache[TrajectoryType.bounceA9ToF.value] = calcTrajectory("Bounce A9-F", 0.3, 0.4, true, 
            new Pose2d(6.858, 3.81, new Rotation2d(90.0)),
            List.of(),
            new Pose2d(8.382, 2.286, new Rotation2d(Math.toRadians(180.0)))
        );
    }


    /**
     * Builds a single trajectory based on the parameters passed in:
     * @param trajName name of the trajectory
     * @param maxVelRatio maximum velocity multiplier between 0 and 1
     * @param maxAccelRatio maximum acceleration multiplier between 0 and 1
     * @param setReversed true = robot drives backwards, false = robot drives forwards
     * @param startPose Pose2d starting position (coordinates and angle)
     * @param interriorWaypoints List of Translation 2d waypoints (just coordinates)
     * @param endPose Pose2d ending position (coordinates and angle)
     * @return trajectory that is generated
     */
    private Trajectory calcTrajectory(String trajName, double maxVelRatio, double maxAccelRatio, 
        boolean setReversed, Pose2d startPose, List<Translation2d> interriorWaypoints, Pose2d endPose) {
		Trajectory trajectory = null;
		DifferentialDriveKinematics driveKinematics = TrajectoryUtil.getDriveKinematics();
	
    	try {

			log.writeLogEcho(true, "TrajectoryGeneration", trajName, 
				"trackWidth",DriveConstants.TRACK_WIDTH,
				"maxVoltage", DriveConstants.MAX_VOLTAGE_IN_TRAJECTORY, 
				"kS", DriveConstants.kS, 
				"kV", DriveConstants.kV, 
				"kA", DriveConstants.kA,
				"maxSpeed", DriveConstants.kMaxSpeedMetersPerSecond,
				"maxAcceleration", DriveConstants.kMaxAccelerationMetersPerSecondSquared);

			// Create a voltage constraint to ensure we don't accelerate too fast
			DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA), 
				driveKinematics,
				DriveConstants.MAX_VOLTAGE_IN_TRAJECTORY);

			// Create config for trajectory
			TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond * maxVelRatio,
				DriveConstants.kMaxAccelerationMetersPerSecondSquared * maxAccelRatio)
				.setKinematics(driveKinematics)
				.addConstraint(autoVoltageConstraint)
				.setReversed(setReversed);			// Set to true if robot is running backwards

            // Generate the trajectory
			trajectory = TrajectoryGenerator.generateTrajectory(
				startPose, interriorWaypoints, endPose, config);

			// debug logging
			TrajectoryUtil.dumpTrajectory(trajectory, log);

		} catch (Exception e) {
			log.writeLogEcho(true, "TrajectoryGeneration", trajName, 
				"ERROR in calcTrajectory", e.toString(),"exception",e);
		}

		if (trajectory != null) {
			log.writeLogEcho(true, "TrajectoryGeneration", trajName, "SUCCESS", true);
		};
	
		return trajectory;
	}

}
