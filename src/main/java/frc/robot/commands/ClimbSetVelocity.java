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

public class ClimbSetVelocity extends CommandBase {
  private Climb climb;
  private FileLog log;
  private double velocity, position, timeRemaining;

  /**
   * Set both climb arm velocities.
   * This command ends when the both climb arms gets within the tolerance of the target position.
   * However, it will stop applying power to an arm if it reaches the target position sooner.
   * @param velocity velocity (inches/second)
   * @param position target position (inches)
   * @param climb climb subsystem
   * @param log filelog utility
   */
  public ClimbSetVelocity(double velocity, double position, Climb climb, FileLog log) {
    this.climb = climb;
    this.log = log;
    this.velocity = velocity;
    this.position = position;
    addRequirements(climb);
  }

  /**
   * Set both climb arm velocities with the default velocity.
   * This command ends when the both climb arms gets within the tolerance of the target position.
   * However, it will stop applying power to an arm if it reaches the target position sooner.
   * @param down true = climb going down (lifting), false = climb going up (latching)
   * @param position target position (inches)
   * @param climb climb subsystem
   * @param log filelog utility
   */
  public ClimbSetVelocity(boolean down, double position, Climb climb, FileLog log) {
    this.climb = climb;
    this.log = log;
    if (down) this.velocity = ClimbConstants.defaultVelocity;
    else this.velocity = -1 * ClimbConstants.defaultVelocity;
    this.position = position;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "ClimbSetVelocity", "Init", "Target ips", velocity, "Target pos", position);
    timeRemaining = DriverStation.getInstance().getMatchTime();
    // if it is the last 30 seconds of the match and the piston is extended, set right motor velocity
    if (/*timeRemaining <= 30 && */climb.climbPistonsGetPosition()) {
      climb.unlockClimbPiston(true);
      climb.climbMotorsSetVelocity(velocity);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.climbMotorRightSetPercentOutput(0);
    climb.climbMotorLeftSetPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if it is the last 30 seconds of the match and the piston is extended, check for being at target position
    if (/*timeRemaining <= 30 && */climb.climbPistonsGetPosition()) {
      if (Math.abs(climb.getRightEncoderInches() - position) <= ClimbConstants.positionTolerance || 
        Math.abs(climb.getLeftEncoderInches() - position) <= ClimbConstants.positionTolerance) return true;
      else return false;
    } else return true;
  }
}
