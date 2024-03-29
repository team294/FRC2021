package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.SearchType;
import frc.robot.commands.AutoGalacticSearch;
import frc.robot.commands.DriveFollowTrajectory;
import frc.robot.commands.AutoNavBouncePath;
import frc.robot.commands.AutoOpponentTrenchPickup;
import frc.robot.commands.AutoShootBackup;
import frc.robot.commands.AutoShootForward;
import frc.robot.commands.AutoShortShot;
import frc.robot.commands.AutoOwnTrenchPickup;
import frc.robot.commands.AutoTrussPickup;
import frc.robot.commands.Wait;
import frc.robot.subsystems.*;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;
import frc.robot.Constants.StopType;


/**
 * Selects the auto routine based upon input from the shuffleboard
 */
public class AutoSelection {

	public static final int OPPONENT_TRENCH_PICKUP = 0;
	public static final int SHOOT_BACKUP = 1;
	public static final int TRUSS_PICKUP = 2;
	public static final int OWN_TRENCH_PICKUP = 3;
	public static final int SHOOT_FORWARD = 4;
	public static final int SHORT_SHOT = 5;
	public static final int BOUNCE_PATH = 6;
	public static final int SLALOM_PATH = 7;
	public static final int BARREL_RACING = 8;
	public static final int GALACTIC_CAMERA = 9;
	public static final int RED_A = 10;
	public static final int BLUE_A = 11;
	public static final int RED_B = 12;
	public static final int BLUE_B = 13;
	
	private TrajectoryCache trajectoryCache;
	
	/**
	 * AutoSelection constructor for command group
	 * Sets up autoPlan widget 
	 */  	
	public AutoSelection(TrajectoryCache trajectoryCache, FileLog log) {
		this.trajectoryCache = trajectoryCache;
	}

	/**
	 * Gets the auto command based upon input from the shuffleboard
	 * 
	 * @param waitTime   The time to wait before starting the auto routines
	 * @param useVision  true = use vision, false = don't use vision
	 * @param autoPlan   The autoplan to run
	 * @param driveTrain The driveTrain that will be passed to the auto command
	 * @param shooter
	 * @param feeder
	 * @param hopper
	 * @param intake
	 * @param limeLightGoal
	 * @param log        The filelog to write the logs to
	 * @param led
	 * @param CoordType
	 * @return the command to run
	 */
	public Command getAutoCommand(double waitTime, boolean useVision, Integer autoPlan, DriveTrain driveTrain,
			Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LimeLightGoal limeLightGoal, LimeLightBall limeLightBall, FileLog log, LED led) {
		Command autonomousCommand = null;
		Trajectory trajectory;

		if (autoPlan == OPPONENT_TRENCH_PICKUP && trajectoryCache.cache[TrajectoryType.opponentTrenchPickup.value] != null) {
			log.writeLogEcho(true, "AutoSelect", "run TrenchFromRight");
			trajectory = trajectoryCache.cache[TrajectoryType.opponentTrenchPickup.value];
			autonomousCommand = new AutoOpponentTrenchPickup(waitTime, useVision, trajectory, driveTrain, limeLightGoal, log, shooter, feeder, hopper, intake, led);
		}

		if (autoPlan == SHOOT_BACKUP) {
			log.writeLogEcho(true, "AutoSelect", "run ShootBackup");
			autonomousCommand = new AutoShootBackup(waitTime, useVision, driveTrain, limeLightGoal, log, shooter, feeder, hopper, intake, led);
		}

		if (autoPlan == SHOOT_FORWARD) {
			log.writeLogEcho(true, "AutoSelect", "run ShootForward");
			autonomousCommand = new AutoShootForward(waitTime, useVision, driveTrain, limeLightGoal, log, shooter, feeder, hopper, intake, led);
		}

		if (autoPlan == TRUSS_PICKUP) {
			log.writeLogEcho(true, "AutoSelect", "run TrussPickup");
			autonomousCommand = new AutoTrussPickup(waitTime, useVision, driveTrain, limeLightGoal, log, shooter, feeder, hopper, intake, led);
		}

		if (autoPlan == OWN_TRENCH_PICKUP) {
			log.writeLogEcho(true, "AutoSelect", "run OwnTrenchPickup");
			autonomousCommand = new AutoOwnTrenchPickup(waitTime, useVision, driveTrain, limeLightGoal, log, shooter, feeder, hopper, intake, led);
		} 

		if (autoPlan == SHORT_SHOT){
			log.writeLogEcho(true, "AutoSelect", "run ShortShot");
			autonomousCommand = new AutoShortShot(waitTime, useVision, driveTrain, limeLightGoal, log, shooter, feeder, hopper, intake, led);
		}

		if (autoPlan == BOUNCE_PATH){
			log.writeLogEcho(true, "AutoSelect", "run BouncePath");
			autonomousCommand = new AutoNavBouncePath(trajectoryCache, driveTrain, log);
		}

		if (autoPlan == SLALOM_PATH){
			log.writeLogEcho(true, "AutoSelect", "run SlalomPath");
			autonomousCommand = new DriveFollowTrajectory(CoordType.kAbsoluteResetPose, StopType.kBrake, trajectoryCache.cache[TrajectoryType.slalom.value], 
				driveTrain, log);
		}

		if (autoPlan == BARREL_RACING){
			log.writeLogEcho(true, "AutoSelect", "run BarrelRacing");
			autonomousCommand = new DriveFollowTrajectory(CoordType.kAbsoluteResetPose, StopType.kBrake, trajectoryCache.cache[TrajectoryType.barrelRacing.value], 
				driveTrain, log);
		}

		if (autoPlan == GALACTIC_CAMERA) {
			log.writeLogEcho(true, "AutoSelect", "run Galactic Camera");
			autonomousCommand = new AutoGalacticSearch(limeLightBall.getGalacticPath(), trajectoryCache, driveTrain, log, intake);
		}

		if (autoPlan == RED_A && trajectoryCache.cache[TrajectoryType.galacticRedA.value] != null) {
			log.writeLogEcho(true, "AutoSelect", "run Galactic Red A");
			autonomousCommand = new AutoGalacticSearch(SearchType.kRedA, trajectoryCache, driveTrain, log, intake);
		}

		if (autoPlan == BLUE_A && trajectoryCache.cache[TrajectoryType.galacticBlueA.value] != null) {
			log.writeLogEcho(true, "AutoSelect", "run Galactic Blue A");
			autonomousCommand = new AutoGalacticSearch(SearchType.kBlueA, trajectoryCache, driveTrain, log, intake);
		}

		if (autoPlan == RED_B && trajectoryCache.cache[TrajectoryType.galacticRedB.value] != null) {
			log.writeLogEcho(true, "AutoSelect", "run Galactic Red B");
			autonomousCommand = new AutoGalacticSearch(SearchType.kRedB, trajectoryCache, driveTrain, log, intake);
		}

		if (autoPlan == BLUE_B && trajectoryCache.cache[TrajectoryType.galacticBlueB.value] != null) {
			log.writeLogEcho(true, "AutoSelect", "run Galactic Blue B");
			autonomousCommand = new AutoGalacticSearch(SearchType.kBlueB, trajectoryCache, driveTrain, log, intake);
		}
		
		if (autonomousCommand == null) {
			log.writeLogEcho(true, "AutoSelect", "No autocommand found");
			autonomousCommand = new Wait(1);
		}

		return autonomousCommand;
	}

}
