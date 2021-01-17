/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class ClimbPistonsSetPosition extends CommandBase {
  private Climb climb;
  private FileLog log;
  private boolean extend;
  private double matchTime;
  
  /**
   * Set piston position of both climb arms.
   * This command immediately ends.
   * @param extend true = extend pistons, false = retract pistons
   * @param climb climb subsystem
   */
  public ClimbPistonsSetPosition(boolean extend, Climb climb, FileLog log) {
    this.climb = climb;
    this.log = log;
    this.extend = extend;
    this.matchTime = DriverStation.getInstance().getMatchTime();
    addRequirements(climb);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "ClimbPistonsSetPosition", "Init", "Piston Status", (extend) ? "Extend" : "Retract");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (matchTime <= 30 && climb.getLeftEncoderInches() > -2 && climb.getRightEncoderInches() > -2)
      climb.climbPistonsSetPosition(extend);
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
