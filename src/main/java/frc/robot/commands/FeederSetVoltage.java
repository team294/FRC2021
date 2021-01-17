/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.FileLog;

public class FeederSetVoltage extends CommandBase {
  private Feeder feeder;
  private FileLog log;
  private double voltage;

  /**
   * Set feeder voltage using parameter voltage.
   * This command never ends.
   * @param voltage voltage
   * @param feeder feeder subsystem to use
   */
  public FeederSetVoltage(double voltage, Feeder feeder, FileLog log) {
    this.feeder = feeder;
    this.log = log;
    this.voltage = voltage;
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.feederSetVoltage(voltage);
    log.writeLog(false, "FeederSetVoltage", "Init", "TargetVolts", voltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.feederSetVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
