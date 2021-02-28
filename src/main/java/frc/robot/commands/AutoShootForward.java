/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TargetType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

public class AutoShootForward extends SequentialCommandGroup {


  /**
   * Creates a command group that waits a specified time, shoots and then moves forward out of the way. 
   * Can start anywhere on auto line between left most pole from driver perspective and close to right edge of the field, needs to be semi lined up with target
   * 
   * @param waitTime       the time in seconds to wait before starting
   * @param driveTrain     drivetrain subsystem to use
   * @param limeLight      limelightGoal subsystem to use
   * @param log            log subsystem to use
   * @param shooter        shooter subsystem to use
   * @param feeder         feeder subsystem to use
   * @param hopper         hopper subsystem to use
   * @param intake         intake subsystem to use
   * @param led            led subsystem to use
   */
  public AutoShootForward(double waitTime, boolean useVision, DriveTrain driveTrain, LimeLightGoal limeLight, FileLog log, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {

    // can start anywhere on auto line between left most pole from driver perspective and close to right edge of the field, needs to be semi lined up with target

    addCommands(

      // wait before starting
      new Wait(waitTime),

      new ConditionalCommand(
        // With Vision
        sequence( 
          deadline(
            new DriveTurnGyro(TargetType.kVision, 0, 450, 200, 0.8, driveTrain, limeLight, log).withTimeout(DriveConstants.maxSecondsForTurnGyro), 
            new ShooterSetPID(true, false, shooter, limeLight, led, log), // start shooter
            new IntakePistonSetPosition(true, intake, log) // deploy intake piston
          ),

          deadline(
            new WaitForPowerCells(3, shooter, log).withTimeout(5), 
            new ShootSequence(true, shooter, feeder, hopper, intake, limeLight, led, log) 
          )
        ), 

        // Without Vision
        sequence(
          new IntakePistonSetPosition(true, intake, log),
        
          deadline(
            new WaitForPowerCells(3, shooter, log).withTimeout(7), // wait for 3 powercells
            new ShooterHoodPistonSequence(true, false, shooter, log), // change hood
            new ShootSequence(2500, shooter, feeder, hopper, intake, led, log)
          )
        ),
        () -> useVision && limeLight.seesTarget()
      ),

      parallel( 
        // stop all motors
        new ShootSequenceStop(shooter, feeder, hopper, intake, led, log).withTimeout(0.1), 
        // go forward 2 meters to get off auto line
        new DriveStraight(2,TargetType.kRelative, 0, 2.61, 3.8, true, driveTrain, limeLight, log)
      ),
      parallel(
        new IntakePistonSetPosition(false, intake, log),
        new DriveTurnGyro(TargetType.kRelative, 170, 450.0, 200, true, 2, driveTrain, limeLight, log)
      )
    );
  }
}
