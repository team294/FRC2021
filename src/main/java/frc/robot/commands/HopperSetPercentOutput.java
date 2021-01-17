/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.utilities.FileLog;

public class HopperSetPercentOutput extends CommandBase {
  private Hopper hopper;
  private FileLog log;
  private double percent;
  private boolean end;

  /**
   * Set hopper percent output using parameter percent.
   * @param percent percent output (0 to 1)
   * @param end true = end command immediately, false = never end command
   * @param hopper hopper subsystem
   */
  public HopperSetPercentOutput(double percent, boolean end, Hopper hopper, FileLog log) {
    this.hopper = hopper;
    this.percent = percent;
    this.log = log;
    this.end = end;
    addRequirements(hopper);
  }

  /**
   * Set hopper percent output using default percent from constants.
   * This command immediately ends.
   * @param end true = end command immediately, false = never end command
   * @param hopper hopper subsystem
   */
  public HopperSetPercentOutput(boolean end, Hopper hopper, FileLog log) {
    this.hopper = hopper;
    this.log = log;
    this.percent = HopperConstants.hopperDefaultPercentOutput;
    this.end = end;
    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hopper.hopperSetPercentOutput(percent);
    log.writeLog(false, "HopperSetPercentOutput", "Init", "Target %", percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) hopper.hopperSetPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (end) return true;
    else return false;
  }
}
