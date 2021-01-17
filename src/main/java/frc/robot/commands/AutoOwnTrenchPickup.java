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
public class AutoOwnTrenchPickup extends SequentialCommandGroup {

// start with front two wheels on auto line drive frame 14 in from the right wall driver perspective

  public AutoOwnTrenchPickup(double waitTime, boolean useVision, DriveTrain driveTrain, LimeLight limeLight, FileLog log, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {
    
    addCommands(
      
      new DriveZeroGyro(driveTrain, log),
      new Wait(waitTime),

      new LogEnableFastLogging(true, limeLight, log),

      deadline(
        new DriveTurnGyro(TargetType.kAbsolute, 22, 150.0, 200, 2, driveTrain, limeLight, log).withTimeout(1.5),
        new ShooterSetPID(true, false, shooter, limeLight, led, log), // start shooter
        new IntakePistonSetPosition(true, intake, log) // deploy intake piston
      ),
      
      // if the intake blocks the limeLight while deploying, we want to wait until we see the target before moving on
      new ConditionalCommand(
        new LimeLightWaitForTarget(limeLight).withTimeout(2),
        new Wait(0),
        () -> useVision
      ),

      new LogEnableFastLogging(false, limeLight, log),

      new ConditionalCommand(
        sequence(
          new DriveTurnGyro(TargetType.kVision, 0, 150.0, 200, 1, driveTrain, limeLight, log).withTimeout(2), // turn towards target w/ vision

          deadline(
            new WaitForPowerCells(3, shooter, log).withTimeout(3), // wait for 3 power cells to be shot
            new ShootSequence(true, shooter, feeder, hopper, intake, limeLight, led, log) // start shooter
          ),
          
          new ShootSequenceStop(shooter, feeder, hopper, intake, led, log).withTimeout(0.1), // stop all motors

          new DriveTurnGyro(TargetType.kAbsolute, 180, 150.0, 200, 1, driveTrain, limeLight, log).withTimeout(2.0) // turn towards trench
        ),
        sequence(
          new FileLogWrite(false, false, "AutoOwnTrenchPickup", "1st shot doesn't see target", log),
          new Wait(15)          
        ),
        () -> useVision && limeLight.seesTarget()
      ),

      deadline( // drive down trench with intake
            new DriveStraight(4.7494, TargetType.kAbsolute, 180.0, 2.088, 3.8, true, driveTrain, limeLight, log),
            new IntakeSequence(intake, log)
      ),
          
      new DriveTurnGyro(TargetType.kAbsolute, 15, 150.0, 200, 4, driveTrain, limeLight, log).withTimeout(2.0),

      new ConditionalCommand(
        // with Vision
        new SequentialCommandGroup(
          deadline(
            new DriveTurnGyro(TargetType.kVision, 0, 150.0, 200, 1, driveTrain, limeLight, log).withTimeout(2), // turn towards target w/ vision
            new ShooterSetPID(true, false, shooter, limeLight, led, log) // start shooter
          ),
    
          deadline(
            new WaitForPowerCells(3, shooter, log), // wait for 3 power cells to be shot
            new ShootSequence(true, shooter, feeder, hopper, intake, limeLight, led, log) // start shooter
          ),
          
          new ShootSequenceStop(shooter, feeder, hopper, intake, led, log).withTimeout(0.1) // stop all motors
        ), 
        
        // Without vision, pickup 2 balls in trench, but dont shoot
        new SequentialCommandGroup(
          new ShootSequenceStop(shooter, feeder, hopper, intake, led, log),
          new DriveTurnGyro(TargetType.kAbsolute, 180, 150.0, 200, 2, driveTrain, limeLight, log),
          deadline(
            new DriveStraight(1.956, TargetType.kAbsolute, 179, 2.088, 3.8, true, driveTrain, limeLight, log),
            new IntakeSequence(intake, log)
          ),
          new DriveTurnGyro(TargetType.kAbsolute, -15, 150, 200, true, 2, driveTrain, limeLight, log)
        ), () -> useVision && limeLight.seesTarget()
      )

    );
  }
}
