package frc.robot.utilities;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import frc.robot.Constants.DriveConstants;

/**
 * Utility class for Trajectories
 */
public class TrajectoryUtil {
		
	/**
	 * Return the drive kinematics for this drive train
	 */  
	public static DifferentialDriveKinematics getDriveKinematics() {
		return new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH);
	}

	/**
	 * Dump the trajectory to the log
	 */  
 	public static void dumpTrajectory(Trajectory trajectory, FileLog log) {
		for (State s : trajectory.getStates()) {
			var pose = s.poseMeters;
			var translation = pose.getTranslation();
			var rotation = pose.getRotation();

			log.writeLog(true, "TrajectoryGeneration", "Trajectory", 
				"Time", s.timeSeconds, 
				"x", translation.getX(), 
				"y", translation.getY(), 
				"degrees", rotation.getDegrees(), 
				"velocityMetersPerSec", s.velocityMetersPerSecond);
			
		}
	}

	

}
