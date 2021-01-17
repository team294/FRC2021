/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;
import frc.robot.utilities.FileLog;

public class ClimbSetPercentOutput extends CommandBase {
  private Climb climb;
  private FileLog log;
  private double percent, timeRemaining;

  /**
   * Set percent output of both climb motors.
   * This command never ends.
   * @param percent percent output (0 to 1)
   * @param climb climb subsystem
   */
  public ClimbSetPercentOutput(double percent, Climb climb, FileLog log) {
    this.climb = climb;
    this.log = log;
    this.percent = percent;
    timeRemaining = DriverStation.getInstance().getMatchTime();
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "ClimbSetPercentOut", "Init", "Target %", percent);
    // if it is the last 30 seconds of the match and the piston is extended, set left and right motor perecnt output
    if (/*timeRemaining <= 30 && */climb.climbPistonsGetPosition()) {
      climb.unlockClimbPiston(true);
      climb.climbMotorsSetPercentOutput(percent);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.climbMotorsSetPercentOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if it is the last 30 seconds of the match and the piston is extended, do not finish
    /* if (timeRemaining <= 30 && climb.climbPistonsGetPosition()) return false; */
    // else return true;
    if (climb.getLeftEncoderInches() >= ClimbConstants.maxHeight && percent > 0
      /*|| climb.getRightEncoderInches() >= ClimbConstants.maxHeight*/) return true;
    else return false;
  }
}
