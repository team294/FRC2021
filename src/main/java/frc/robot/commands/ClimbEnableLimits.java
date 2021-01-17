/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;
import frc.robot.utilities.FileLog;

public class ClimbEnableLimits extends CommandBase {
  private Climb climb;
  private FileLog log;
  private boolean enable;

  public ClimbEnableLimits(boolean enable, Climb climb, FileLog log) {
    this.climb = climb;
    this.log = log;
    this.enable = enable;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "ClimbEnableLimits", "Init", "Enable", enable);
    climb.enableHardLimits(enable);
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
