/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.FileLog;

public class FeederSetPID extends CommandBase {
  private Feeder feeder;
  private FileLog log;
  private double rpm;
  private boolean fromShuffleboard;
  private Timer timer;

  /**
   * Set feeder PID using parameter RPM.
   * This command ends when feeder RPM is within tolerance.
   * @param rpm setpoint, in RPM
   * @param feeder feeder subsystem
   */
  public FeederSetPID(double rpm, Feeder feeder, FileLog log) {
    this.feeder = feeder;
    this.log = log;
    this.rpm = rpm;
    this.fromShuffleboard = false;
    this.timer = new Timer();
    addRequirements(feeder);
  }

  /**
   * Set feeder PID using RPM from shuffleboard.
   * This command ends when feeder RPM is within tolerance.
   * @param feeder feeder subsystem
   */
  public FeederSetPID(Feeder feeder, FileLog log) {
    this.feeder = feeder;
    this.log = log;
    this.rpm = 0;
    this.fromShuffleboard = true;
    this.timer = new Timer();
    addRequirements(feeder);

    if(SmartDashboard.getNumber("Feeder Manual SetPoint RPM", -9999) == -9999)
      SmartDashboard.putNumber("Feeder Manual SetPoint RPM", 2000);
  }

  /**
   * Set feeder PID using either RPM from shuffleboard or default RPM from constants.
   * This command ends when feeder RPM is within tolerance.
   * @param feeder feeder subsystem
   */
  public FeederSetPID(boolean fromShuffleboard, Feeder feeder, FileLog log) {
    this.feeder = feeder;
    this.rpm = FeederConstants.feederDefaultRPM;
    this.fromShuffleboard = fromShuffleboard;
    this.timer = new Timer();
    addRequirements(feeder);

    if(SmartDashboard.getNumber("Feeder Manual SetPoint RPM", -9999) == -9999)
      SmartDashboard.putNumber("Feeder Manual SetPoint RPM", 2000);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(fromShuffleboard) rpm = SmartDashboard.getNumber("Feeder Manual SetPoint RPM", 2000);
    timer.reset();
    timer.start();
    feeder.setFeederPID(rpm);
    log.writeLog(false, "FeederSetPID", "Init", "TargetRPM", rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) feeder.feederSetVoltage(0);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(0.1) && Math.abs(feeder.getFeederPIDError()) < RobotConstants.pidErrorTolerance) return true;
    else return false;
  }
}
