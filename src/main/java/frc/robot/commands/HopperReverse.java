/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.utilities.FileLog;

public class HopperReverse extends CommandBase {
  private Hopper hopper;
  private FileLog log;
  private Timer timerReverse, timerForward;

  /**
   * Pulse hopper by reversing it periodically.
   * This command never ends.
   * @param hopper hopper subsystem
   */
  public HopperReverse(Hopper hopper, FileLog log) {
    this.hopper = hopper;
    this.log = log;
    this.timerReverse = new Timer();
    this.timerForward = new Timer();
    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timerReverse.reset();
    timerForward.reset();
    timerReverse.start(); // start reverse timer, assuming that hopper is currently running reverse

    log.writeLog(false, "HopperReverse", "Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if hopper has been running forward for 1 second, run reverse, reset forward timer, and start reverse timer
    if (timerForward.hasElapsed(1)) {
      timerForward.stop();
      timerForward.reset();
      timerReverse.start();
      hopper.hopperSetPercentOutput(-1 * HopperConstants.hopperDefaultPercentOutput);
    }
    // if hopper has been running reverse for 0.5 seconds, run forward, reset reverse timer, and start forward timer
    else if (timerReverse.hasElapsed(0.5)) {
      timerReverse.stop();
      timerReverse.reset();
      timerForward.start();
      hopper.hopperSetPercentOutput(HopperConstants.hopperDefaultPercentOutput);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timerReverse.stop();
    timerForward.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
