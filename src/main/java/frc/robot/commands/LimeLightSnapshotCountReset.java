/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLight;
import frc.robot.utilities.FileLog;

public class LimeLightSnapshotCountReset extends CommandBase {
  /**
   * Resets number tracking how many snapshots we have taken so far
   */
  LimeLight limeLight;
  FileLog log;

  public LimeLightSnapshotCountReset(LimeLight limeLight, FileLog log) {
    this.limeLight = limeLight;
    this.log = log;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "LimelightSnapshotCountReset", "Init");
    limeLight.resetSnapshotCount();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
