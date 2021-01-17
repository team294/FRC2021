/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TargetType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShootBackup extends SequentialCommandGroup {
  /**
   * Creates a new AutoShootBackup.
   */
  public AutoShootBackup(double waitTime, boolean useVision, DriveTrain driveTrain, LimeLight limeLight, FileLog log, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {

    // can start anywhere on auto line between left most pole from driver perspective and close to right edge of the field, needs to be semi lined up with target
    // one front wheel on line, one behind
    addCommands(

      new Wait(waitTime),
      new ConditionalCommand(
        // With Vision
        sequence(
          deadline(
            new DriveTurnGyro(TargetType.kVision, 0, 450, 200, 0.8, driveTrain, limeLight, log).withTimeout(2), // turn towards target w/ vision
            new ShooterSetPID(true, false, shooter, limeLight, led, log),
            new ShooterHoodPistonSequence(true, false, shooter, log)
          ),

          deadline(
          new WaitForPowerCells(3, shooter, log).withTimeout(5), // wait for 3 power cells to be shot
          new ShootSequence(true, shooter, feeder, hopper, intake, limeLight, led, log) // start shooter
          )
        ), 
          // Without Vision
          sequence(
            new IntakePistonSetPosition(true, intake, log), // deploy intake piston
            
            deadline(
              new WaitForPowerCells(3, shooter, log).withTimeout(7), // wait for 3 powercells to be shot
              new ShooterHoodPistonSequence(true, false, shooter, log), // change hood
              new ShootSequence(2500, shooter, feeder, hopper, intake, led, log) // shoot
            )
          ), 
          () -> useVision && limeLight.seesTarget()
        ),
      
      

      parallel(
        new ShootSequenceStop(shooter, feeder, hopper, intake, led, log).withTimeout(0.1), // stop all motors
        new DriveStraight(-1, TargetType.kRelative, 0.0, 2.61, 3.8, true, driveTrain, limeLight, log) // back up 1 meter to get off auto line
      ),
      parallel(
        new IntakePistonSetPosition(false, intake, log),
        new DriveTurnGyro(TargetType.kRelative, 170, 450.0, 200, true, 2, driveTrain, limeLight, log)
      )
    );
  }
}
