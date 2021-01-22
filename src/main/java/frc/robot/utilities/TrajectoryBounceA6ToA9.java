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
 * Trajectory methods for going to the trench from the center position
 */
public class TrajectoryBounceA6ToA9 {

	/**
	* Calculate the trajectory used to get the balls from the trench starting from the center
	* (the center is directly in front of the target)
	*/
	public static Trajectory calcTrajectory(FileLog log) {
		Trajectory trajectory = null;
		DifferentialDriveKinematics driveKinematics = TrajectoryUtil.getDriveKinematics();
	
    	try {

			log.writeLogEcho(true, "TrajectoryGeneration", "Test", 
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
			TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond * 0.4,
				DriveConstants.kMaxAccelerationMetersPerSecondSquared * 0.4)
				.setKinematics(driveKinematics)
				.addConstraint(autoVoltageConstraint)
				.setReversed(false);			// Set to true if robot is running backwards

			// drive from line to trench (assumes starting directly in front and facing target)		  
			// the trajectory to follow (all units in meters)
			// start at the origin facing the +X direction
			// Y is distance forward
			// firstBallY = -1.4
			// firstBallX = -3.1
			// distance between first and last ball = 1.82
			trajectory = TrajectoryGenerator.generateTrajectory(
				new Pose2d(4.572, 3.81, new Rotation2d(Math.toRadians(-90.0))),
				List.of(
					new Translation2d(4.572, 1.524),
					new Translation2d(5.715, 0.762),
					new Translation2d(6.858, 1.524)
				),
				new Pose2d(6.858, 3.81, new Rotation2d(Math.toRadians(90.0))), config);

			// debug logging
			TrajectoryUtil.dumpTrajectory(trajectory, log);

		} catch (Exception e) {
			log.writeLogEcho(true, "TrajectoryGeneration", "OpponentTrench", 
				"ERROR in calcTrajectory", e.toString(),"exception",e);
		}

		if (trajectory != null) {
			log.writeLogEcho(true, "TrajectoryGeneration", "OpponentTrench", "SUCCESS", true);
		};
	
		return trajectory;
	}

	

}
