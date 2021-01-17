/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.TemperatureCheck;

import static frc.robot.Constants.RobotConstants.*;
import static frc.robot.Constants.FeederConstants.*;

public class Feeder extends SubsystemBase implements Loggable {
  private final WPI_TalonFX feederMotor = new WPI_TalonFX(canFeederMotor); // 9:1 gear ratio

  private double measuredVelocityRaw, measuredRPM, feederRPM, setPoint;
  private double kP, kI, kD, kFF, kMaxOutput, kMinOutput; // PID terms
  private int timeoutMs = 0;  // was 30, changed to 0 for testing
  private double ticksPer100ms = 600.0 / 2048.0; // convert raw units to RPM (2048 ticks per revolution)
  private double ff, p, i, d; // for shuffleboard
  private boolean fastLogging = false;

  private FileLog log;
  private TemperatureCheck tempCheck;

  public Feeder(FileLog log, TemperatureCheck tempCheck) {
    this.log = log; // save reference to the fileLog
    this.tempCheck = tempCheck;

    feederMotor.configFactoryDefault();
    feederMotor.setInverted(false);
    feederMotor.setNeutralMode(NeutralMode.Coast);
    feederMotor.configClosedloopRamp(0.1); // seconds from neutral to full
    feederMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs);
    feederMotor.setSensorPhase(false);
    feederMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);

    // PID coefficients
    kP = 0.1;
    kI = 0;
    kD = 0;
    kFF = 0.047;
    kMaxOutput = 1; 
    kMinOutput = 0;

    // set PID coefficients
    feederMotor.config_kP(0, kP, timeoutMs);
    feederMotor.config_kI(0, kI, timeoutMs);
    feederMotor.config_kD(0, kD, timeoutMs);
    feederMotor.config_kF(0, kFF, timeoutMs);
    feederMotor.configPeakOutputForward(kMaxOutput);
    feederMotor.configPeakOutputReverse(kMinOutput);
    feederMotor.setSensorPhase(false);

    SmartDashboard.putNumber("Feeder SetPoint RPM", feederRPM);
    SmartDashboard.putNumber("Feeder Manual SetPoint RPM", feederRPM);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Feeder P", kP);
    SmartDashboard.putNumber("Feeder I", kI);
    SmartDashboard.putNumber("Feeder D", kD);
    SmartDashboard.putNumber("Feeder FF", kFF);
  }

  /**
   * @param voltage voltage
   */
  public void feederSetVoltage(double voltage) {
    feederMotor.setVoltage(voltage);
  }

  /**
   * Run feeder in a velocity closed loop mode.
   * If setPoint RPM is 0, this uses Shuffleboard as input.
   * @param feederRPM setPoint RPM
   */  
  public void setFeederPID(double FeederRPM) {
    this.feederRPM = FeederRPM;
    setPoint = this.feederRPM / ticksPer100ms;
    feederMotor.set(ControlMode.Velocity, setPoint);

    SmartDashboard.putNumber("Feeder SetPoint RPM", FeederRPM);
  }

  /**
   * @return output voltage
   */
  public double feederGetVoltage() {
    return feederMotor.getMotorOutputVoltage();
  }

  /**
   * @return PID error, in RPM
   */
  public double getFeederPIDError() {
    return feederRPM - measuredRPM;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    measuredVelocityRaw = feederMotor.getSelectedSensorVelocity(0);
    measuredRPM = measuredVelocityRaw * ticksPer100ms; // converts ticks per 100ms to RPM

    if(fastLogging || log.getLogRotation() == log.FEEDER_CYCLE) {
      updateFeederLog(false); 
    }
    
    if(log.getLogRotation() == log.FEEDER_CYCLE) {

      // read PID coefficients from SmartDashboard
      ff = SmartDashboard.getNumber("Feeder FF", 0);
      p = SmartDashboard.getNumber("Feeder P", 0);
      i = SmartDashboard.getNumber("Feeder I", 0);
      d = SmartDashboard.getNumber("Feeder D", 0);

      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if(ff != kFF) feederMotor.config_kF(0, ff, timeoutMs); kFF = ff;
      if(p != kP) feederMotor.config_kP(0, p, timeoutMs); kP = p;
      if(i != kI) feederMotor.config_kI(0, i, timeoutMs); kI = i;
      if(d != kD) feederMotor.config_kD(0, d, timeoutMs); kD = d;

      SmartDashboard.putNumber("Feeder SetPoint RPM", setPoint * ticksPer100ms);
      SmartDashboard.putNumber("Feeder RPM", measuredRPM);
      SmartDashboard.putNumber("Feeder Encoder", feederMotor.getSelectedSensorPosition());
      SmartDashboard.putNumber("Feeder Voltage", feederMotor.getMotorOutputVoltage());
      SmartDashboard.putNumber("Feeder SetPoint", setPoint);
      SmartDashboard.putNumber("Feeder Error", getFeederPIDError());
      SmartDashboard.putNumber("Feeder PercentOutput", feederMotor.getMotorOutputPercent());
    }
  }

  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Write information about feeder to filelog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
  public void updateFeederLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, "Feeder", "Update Variables", 
      "Motor Volt", feederMotor.getMotorOutputVoltage(), 
      "Motor Amps", feederMotor.getSupplyCurrent(), 
      // "Motor Temp", feederMotor.getTemperature(),
      "Measured RPM", measuredRPM 
    );
  }

  /**
   * Update TemperatureCheck utility with motors that are and are not overheating.
   */
  public void updateOverheatingMotors() {
    if (feederMotor.getTemperature() >= temperatureCheck)
      tempCheck.recordOverheatingMotor("Feeder");
    
    if (feederMotor.getTemperature() < temperatureCheck)
      tempCheck.notOverheatingMotor("Feeder");
  }
}
