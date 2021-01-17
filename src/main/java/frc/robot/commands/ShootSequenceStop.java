/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class ShootSequenceStop extends SequentialCommandGroup {
  /**
   * Stop feeder, hopper, and intake motors. Set shooter to low RPM.
   * @param shooter shooter subsystem
   * @param feeder feeder subsystem
   * @param hopper hopper subsystem
   * @param intake intake subsystem
   * @param led led strip (subsystem)
   */
  public ShootSequenceStop(Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led, FileLog log) {
    addCommands(
      parallel(
        new FileLogWrite(false, false, "ShootSequence", "Stop", log),
        new ShooterSetPID(1200, shooter, led, log),
        new FeederSetVoltage(0, feeder, log),
        new IntakeSetPercentOutput(0, true, intake, log),
        new HopperSetPercentOutput(0, true, hopper, log)
      )
    );
  }
}
