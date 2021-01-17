/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.FileLog;

public class ShooterSetLockPiston extends CommandBase {
  private Shooter shooter;
  private boolean unlock;
  private FileLog log;

  /**
   * Set shooter lock piston position.
   * This command immediately ends.
   * @param unlock true = unlock, false = lock
   * @param shooter shooter subsystem
   */
  public ShooterSetLockPiston(boolean unlock, Shooter shooter, FileLog log) {
    this.shooter = shooter;
    this.unlock = unlock;
    this.log = log;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "IntakeSetPiston", "Init", (unlock) ? "Unlock" : "Lock");
    shooter.setLockPiston(unlock);
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
