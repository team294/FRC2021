/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TargetType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoTrussPickup extends SequentialCommandGroup {
  /**
   * Creates a new AutoTrussPickup.
   */
  public AutoTrussPickup(double waitTime, boolean useVision, DriveTrain driveTrain, LimeLightGoal limeLight, FileLog log, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {
    
    // start with edge of bumpers to the left edge of the center line, intake facing towards truss, line up straight
    
    addCommands(

      new Wait(waitTime),

      new DriveZeroGyro(180, driveTrain, log),

      deadline(
        new DriveStraight(2.08, TargetType.kRelative, 0.0, 2.61, 3.8, true, driveTrain, limeLight, log), // drive to 2 of balls on truss
        new IntakeSequence(intake, log)
        // new IntakePistonSetPosition(true, intake), // deploy intake piston
        // new IntakeSetPercentOutput(intake) // spin intake
      ),

      deadline(
        new DriveStraight(-1, TargetType.kRelative, 0.0, 2.61, 3.8, true, driveTrain, limeLight, log), // back up a short ammount 
        new IntakeSequence(intake, log) // keep intake spinning
      ),

      deadline(
        new DriveTurnGyro(TargetType.kAbsolute, -15, 400, 200, 3, driveTrain, limeLight, log).withTimeout(3), // turn towards general target
        new ShooterSetPID(true, false, shooter, limeLight, led, log)
      ),


      new DriveTurnGyro(TargetType.kVision, 0, 450, 200, 0.8, driveTrain, limeLight, log).withTimeout(2), // turn towards target w/ vision

      new ShooterHoodPistonSequence(true, false, shooter, log),
        
      deadline(
        new WaitForPowerCells(5, shooter, log),  // wait for 5 balls to be shot
        new ShootSequence(true, shooter, feeder, hopper, intake, limeLight, led, log) // shoot
      ),
      
      new ShootSequenceStop(shooter, feeder, hopper, intake, led, log).withTimeout(0.1)// stop all motors
      
    );
  }
}
