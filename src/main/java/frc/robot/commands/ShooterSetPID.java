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
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.FileLog;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LimeLight;

public class ShooterSetPID extends CommandBase {
  private final Shooter shooter;
  private LimeLight limeLight;
  private LED led;
  private FileLog log;
  private double rpm;
  private boolean rpmFromShuffleboard, rpmFromDistance, end;
  private Timer ledTimer;
  private String ledColor = "Blue";
  
  /**
   * Set shooter PID using parameter RPM.
   * This command ends when shooter RPM is within tolerance.
   * @param rpm setpoint, in RPM
   * @param shooter shooter subsystem
   * @param led led strip (subsystem)
   * @param log Filelog subsystem
   */
  public ShooterSetPID(double rpm, Shooter shooter, LED led, FileLog log) {
    this.shooter = shooter;
    this.led = led;
    this.log = log;
    this.rpm = rpm;
    this.rpmFromShuffleboard = false;
    this.rpmFromDistance = false;
    this.end = true;
    this.ledTimer = new Timer();
    addRequirements(shooter);
  }

  /**
   * Set shooter PID using parameter RPM.
   * @param end true = end command when shooter is at setpoint rpm, false = never end
   * @param rpm setpoint, in RPM
   * @param shooter shooter subsystem
   * @param led led strip (subsystem)
   * @param log filelog utility
   */
  public ShooterSetPID(boolean end, double rpm, Shooter shooter, LED led, FileLog log) {
    this.shooter = shooter;
    this.led = led;
    this.log = log;
    this.rpm = rpm;
    this.rpmFromShuffleboard = false;
    this.rpmFromDistance = false;
    this.end = end;
    this.ledTimer = new Timer();
    addRequirements(shooter);
  }

  /**
   * Turn on the shooter PID using RPM from Shuffleboard.
   * @param rpmFromDistance true = rpm is set with distance from target, false = rpm is set with manual dashboard input
   * @param end true = end command when shooter is at setpoint rpm, false = never end
   * @param shooter shooter subsystem to use
   * @param limeLight limeLight to use
   * @param led led to use
   * @param log filelog utility
   */
  public ShooterSetPID(boolean rpmFromDistance, boolean end, Shooter shooter, LimeLight limeLight, LED led, FileLog log) {
    this.shooter = shooter;
    this.limeLight = limeLight;
    this.led = led;
    this.log = log;
    this.rpm = 0;
    this.rpmFromShuffleboard = true;
    this.rpmFromDistance = rpmFromDistance;
    this.end = end;
    this.ledTimer = new Timer();
    addRequirements(shooter);

    if(SmartDashboard.getNumber("Shooter Manual SetPoint RPM", -9999) == -9999)
      SmartDashboard.putNumber("Shooter Manual SetPoint RPM", 2800);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(rpmFromShuffleboard && !rpmFromDistance) rpm = SmartDashboard.getNumber("Shooter Manual SetPoint RPM", 2800);
    else if(rpmFromDistance) rpm = shooter.distanceFromTargetToRPM(limeLight.getDistance());
    shooter.setShooterPID(rpm);
    ledTimer.reset();
    ledTimer.start();
    log.writeLog(false, "ShooterSetPID", "Init", "TargetRPM", rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!end && rpmFromDistance) {
      if (limeLight.getDistance() == 0) rpm = ShooterConstants.shooterDefaultRPM;
      else rpm = shooter.distanceFromTargetToRPM(limeLight.getDistance());
      shooter.setShooterPID(rpm);
      if(limeLight.canTakeSnapshot()) {
        limeLight.setSnapshot(true);
      }    
    }
    
    SmartDashboard.putString("LED Color", ledColor);

    if(ledTimer.advanceIfElapsed(0.1)) {
      led.setStrip(ledColor, 0.5, 1);
      if(ledColor.equals("Blue")) ledColor = "Black";
      else ledColor = "Blue";
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted && end) shooter.setShooterVoltage(0);
    ledTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!end) return false;

    if (shooter.getShooterPIDError() < RobotConstants.pidErrorTolerance) {
      SmartDashboard.putBoolean("Shooter Up To Speed", true);
      led.setStrip("Blue", 0.5, 1);
      return true;
    } else return false;
  }
}
