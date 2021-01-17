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

public class WaitForPowerCells extends CommandBase {
  private Shooter shooter;
  private FileLog log;
  private int cells;

  /**
   * Wait until parameter number of power cells are shot.
   * @param cells number of power cells to wait for
   * @param shooter shooter subsystem
   */
  public WaitForPowerCells(int cells, Shooter shooter, FileLog log) {
    this.shooter = shooter;
    this.log = log;
    this.cells = cells;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "WaitForPowerCells", "Init", "TargetCell #", cells);
    shooter.setPowerCellsShot(0);
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
    if (cells <= shooter.getPowerCellsShot()){
      shooter.setPowerCellsShot(0);
      return true;
    } 
    else return false;
  }
}
