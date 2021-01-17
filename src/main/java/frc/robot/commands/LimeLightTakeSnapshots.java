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

public class LimeLightTakeSnapshots extends CommandBase {
  /**
   * Take a snapshot of the limelight camera image every cycle either during autonomous
   * or when the robot is enabled
   * Intended to help test snapshot capabilities of the limelight
   */
  LimeLight limeLight;
  FileLog log;

  /**
   * @param limeLight save reference to limeLight
   * @param inAuto true = save snapshots only in auto, false = save snapshots whenever enabled
   */
  public LimeLightTakeSnapshots(LimeLight limeLight, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLight = limeLight;
    this.log = log;
    addRequirements(limeLight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "LimelightTakeSnapshots", "Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limeLight.canTakeSnapshot()) {
      limeLight.setSnapshot(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limeLight.setSnapshot(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
