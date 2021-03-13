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
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShortShot extends SequentialCommandGroup {
  
  // start with back two wheels on the line and lined up right infront of target

  public AutoShortShot(double waitTime, boolean useVision, DriveTrain driveTrain, LimeLightGoal limeLight, FileLog log, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(
      new DriveZeroGyro(driveTrain, log),

      deadline(
        new DriveStraight(1.93, TargetType.kAbsolute, 0, 1, 1, true, driveTrain, limeLight, log), // drive up to the target
        new ShooterHoodPistonSequence(false, false, shooter, log), // unlock shooter hood piston and bring hood up
        new IntakePistonSetPosition(true, intake, log), // deploy intake
        new ShooterSetPID(true, 1400, shooter, led, log) // start shooter
      ),

      parallel(
        new WaitForPowerCells(3, shooter, log).withTimeout(8), // shoot 3 balls
        new ShootSequence(1400, shooter, feeder, hopper, intake, led, log)
      ),

      deadline(
        new Wait(0.1),
        new ShootSequenceStop(shooter, feeder, hopper, intake, led, log)
      )
    );
  }
}
