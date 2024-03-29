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
   
    private static int trajectoryCount = 14;
    public Trajectory[] cache = new Trajectory[trajectoryCount];

    public enum TrajectoryType {
        test(0),
        testCurve(1),
        opponentTrenchPickup(2),
        bounceSToA3(3),
        bounceA3ToA6(4),
        bounceA6ToA9(5),
        bounceA9ToF(6),
        slalom(7),
        barrelRacing(8),
        galacticRedA(9),
        galacticBlueA(10),
        galacticRedB(11),
        galacticBlueB(12),
        galacticNull(13);
    
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

        cache[TrajectoryType.bounceSToA3.value] = calcTrajectory("Bounce S-A3", 0.45, 0.6, false, 
            new Pose2d(0.762, 2.286, new Rotation2d(0.0)),
            List.of(),
            new Pose2d(2.286, 3.5, new Rotation2d(Math.toRadians(90.0)))
        );

        cache[TrajectoryType.bounceA3ToA6.value] = calcTrajectory("Bounce A3-6", 0.45, 0.6, true, 
            new Pose2d(2.286, 3.5, new Rotation2d(Math.toRadians(90.0))),
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
            List.of(new Translation2d(2.94, 1.5),
            new Translation2d(3.81, 0.8),
            new Translation2d(4.572, 1.524)),
            new Pose2d(4.572, 3.5, new Rotation2d(Math.toRadians(-90.0)))
        );

        cache[TrajectoryType.bounceA6ToA9.value] = calcTrajectory("Bounce A6-9", 0.45, 0.6, false, 
            new Pose2d(4.572, 3.5, new Rotation2d(Math.toRadians(-90.0))),
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
            new Translation2d(5.715, 0.8),
            new Translation2d(6.858, 1.524)),
            new Pose2d(6.858, 3.5, new Rotation2d(Math.toRadians(90.0)))
        );

        cache[TrajectoryType.bounceA9ToF.value] = calcTrajectory("Bounce A9-F", 0.45, 0.6, true, 
            new Pose2d(6.858, 3.5, new Rotation2d(Math.toRadians(90.0))),
            List.of(),
            new Pose2d(7.8, 2.286, new Rotation2d(Math.toRadians(180.0)))
        );

        cache[TrajectoryType.slalom.value] = calcTrajectory("Slalom", 0.4, 0.5, false, 
            new Pose2d(0.762, 0.762, new Rotation2d(Math.toRadians(0.0))),
            List.of(new Translation2d(1.905, 0.8),
                    new Translation2d(2.85, 2.5),
                    new Translation2d(4.572, 3.048),
                    new Translation2d(6.096, 2.6),
                    new Translation2d(6.858, 1.524),
                    new Translation2d(7.62, 0.762),
                    new Translation2d(8.382, 1.524),
                    new Translation2d(7.45, 2.6),
                    new Translation2d(6.75, 1.524),
                    new Translation2d(6.096, 0.448),
                    new Translation2d(4.572, 0),
                    new Translation2d(2.85, 0.548),
                    new Translation2d(1.905, 2.1)),
            new Pose2d(1.25, 2.486, new Rotation2d(Math.toRadians(180.0)))
        );

        cache[TrajectoryType.barrelRacing.value] = calcTrajectory("Barrel Racing", 0.4, 0.6, false, 
            new Pose2d(1.092, 2.286, new Rotation2d(Math.toRadians(0.0))),
            List.of(new Translation2d(3.81, 2.22),
                    new Translation2d(4.51, 1.52),
                    new Translation2d(3.81, 0.82),
                    new Translation2d(3.11, 1.52),
                    new Translation2d(3.81, 2.22),
                    new Translation2d(6.09, 2.34),
                    new Translation2d(6.79, 3.04),
                    new Translation2d(6.09, 3.74),
                    new Translation2d(5.39, 3.04),
                    new Translation2d(6.0, 2.14),
                    new Translation2d(6.8, 1.32),
                    new Translation2d(7.62, 0.82),
                    new Translation2d(8.5, 1),
                    new Translation2d(7.62, 2.22)),
            new Pose2d(1.04, 2.54, new Rotation2d(Math.toRadians(180.0)))
        );
        
        cache[TrajectoryType.galacticRedA.value] = calcTrajectory("Galactic Red A", 0.4, 0.4, false, 
            new Pose2d(0.762, 3.81, new Rotation2d(Math.toRadians(-45.0))),
            List.of(new Translation2d(2.286, 2.286),
                    new Translation2d(3.81, 1.524),
                    new Translation2d(4.572, 3.81)),
            new Pose2d(8.382, 3.81, new Rotation2d(Math.toRadians(0.0)))
        );

        cache[TrajectoryType.galacticBlueA.value] = calcTrajectory("Galactic Blue A", 0.4, 0.4, false, 
            new Pose2d(0.762, 0.762, new Rotation2d(Math.toRadians(0.0))),
            List.of(new Translation2d(4.572, 0.762),
                    new Translation2d(5.334, 3.048),
                    new Translation2d(6.858, 2.286)),
            new Pose2d(8.382, 0.9, new Rotation2d(Math.toRadians(0.0)))
        );

        cache[TrajectoryType.galacticRedB.value] = calcTrajectory("Galactic Red B", 0.4, 0.4, false, 
            new Pose2d(0.762, 3.81, new Rotation2d(Math.toRadians(-45.0))),
            List.of(new Translation2d(2.286, 3.048),
                    new Translation2d(3.81, 1.524),
                    new Translation2d(5.334, 2.548)),
            new Pose2d(8.382, 3.048, new Rotation2d(Math.toRadians(0.0)))
        );

        cache[TrajectoryType.galacticBlueB.value] = calcTrajectory("Galactic Blue B", 0.4, 0.4, false, 
            new Pose2d(0.762, 0.762, new Rotation2d(Math.toRadians(0.0))),
            List.of(new Translation2d(4.572, 1.524),
                    new Translation2d(6.096, 3.048),
                    new Translation2d(7.62, 1.524)),
            new Pose2d(8.382, 0.762, new Rotation2d(Math.toRadians(-45.0)))
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
