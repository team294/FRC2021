/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.FileLog;

public class IntakeSequence extends SequentialCommandGroup {
  /**
   * Deploy the intake and run intake rollers in.
   * @param intake intake subsystem
   */
  public IntakeSequence(Intake intake, FileLog log) {
    addCommands(
      new ConditionalCommand(
        sequence(
          new IntakePistonSetPosition(true, intake, log),
          new Wait(0.4)
        ), 
        new Wait(0), 
        () -> !intake.intakeGetPiston()),
      new FileLogWrite(false, false, "IntakeSequence", "Init", log),
      new IntakeSetPercentOutput(false, intake, log)
    );
  }
}
