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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.TemperatureCheck;

import static frc.robot.Constants.RobotConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase implements Loggable {
  private final WPI_TalonFX shooterMotorLeft = new WPI_TalonFX(canShooterMotorLeft);
  private final WPI_TalonFX shooterMotorRight = new WPI_TalonFX(canShooterMotorRight);
  private final DoubleSolenoid shooterHoodPiston = new DoubleSolenoid(pcmShooterHoodPistonOut, pcmShooterHoodPistonIn); // piston to open and close hood
  private final Solenoid shooterLockPiston = new Solenoid(pcmShooterLockPiston); // piston to lock hood angle
  private FileLog log; // reference to the fileLog
  private TemperatureCheck tempCheck;
  private final DigitalInput input = new DigitalInput(dioPowerCell);
  private LED led;

  private double measuredVelocityRaw, measuredRPM, shooterRPM, setPoint; // setPoint is in native units
  private double kP, kI, kD, kFF, kMaxOutput, kMinOutput; // PID terms
  private int timeoutMs = 0; // was 30, changed to 0 for testing
  private double ticksPer100ms = 600.0 / 2048.0; // convert raw units to RPM (2048 ticks per revolution)
  private double voltageTarget = 1; // target shooter voltage, used to check if shooter is being set to 0 volts
  private int cellsShot = 0; // counter of total number of power cells shot (reset when shooter is set to 0 volts)
  private int prevCellsShot = 0; // counter of the previous number of power cells shot ()
  private boolean prevCellState = false; // used to indicate whether triggers of photo switch are new power cells
  private boolean fastLogging = false; // true is enabled to run every cycle; false follows normal logging cycles
  
  public Shooter(Hopper hopper, FileLog log, TemperatureCheck tempCheck, LED led) {
    this.log = log; // save reference to the fileLog
    this.tempCheck = tempCheck;
    this.led = led;

    setLockPiston(false);

    shooterMotorLeft.configFactoryDefault();
    shooterMotorRight.configFactoryDefault();
    shooterMotorLeft.setInverted(false);
    shooterMotorRight.setInverted(true);

    // set drives to coast mode and ramp rate
    shooterMotorLeft.setNeutralMode(NeutralMode.Coast);
    shooterMotorRight.setNeutralMode(NeutralMode.Coast);
    shooterMotorLeft.configClosedloopRamp(0.05); //seconds from neutral to full
    shooterMotorRight.configClosedloopRamp(0.05);
    shooterMotorLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs); 
    // shooterMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs);
    shooterMotorLeft.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        
    // PID coefficients initial
    kP = 0.25;
    kI = 0;
    kD = 0; 
    kFF = 0.052; // 0.052 empirically to set to 3000 RPM on prototype shooter (one motor)
    
    // set PID coefficients
    shooterMotorLeft.config_kF(0, kFF, timeoutMs);
    shooterMotorLeft.config_kP(0, kP, timeoutMs);
    shooterMotorLeft.config_kI(0, kI, timeoutMs);
    shooterMotorLeft.config_kD(0, kD, timeoutMs);

    kMaxOutput = 1; 
    kMinOutput = 0; // no need to go negative, a negative number will brake (and possibly break) the motor
    
    shooterMotorLeft.configPeakOutputForward(kMaxOutput);
    shooterMotorLeft.configPeakOutputReverse(kMinOutput);
    shooterMotorLeft.setSensorPhase(false);

    shooterMotorRight.set(ControlMode.Follower, canShooterMotorLeft);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Shooter P", kP);
    SmartDashboard.putNumber("Shooter I", kI);
    SmartDashboard.putNumber("Shooter D", kD);
    SmartDashboard.putNumber("Shooter FF", kFF);
    
    SmartDashboard.putNumber("Shooter SetPoint RPM", shooterRPM);
    SmartDashboard.putNumber("Shooter Manual SetPoint RPM", shooterRPM);
  }

  /**
   * @param voltage voltage
   */
  public void setShooterVoltage(double voltage) {
    shooterMotorLeft.setVoltage(voltage);
    voltageTarget = voltage;
  }

  /**
   * Run shooter in a velocity closed loop mode.
   * If setPoint RPM is 0, this uses Shuffleboard as input.
   * @param shooterRPM setPoint RPM
   */
  public void setShooterPID(double shooterRPM) {
    this.shooterRPM = shooterRPM;
    setPoint = shooterRPM / ticksPer100ms; // setPoint is in ticks per 100ms
    shooterMotorLeft.set(ControlMode.Velocity, setPoint);
    voltageTarget = 1;
    SmartDashboard.putNumber("Shooter SetPoint RPM", shooterRPM);
  }

  /**
   * @param open true = open (retract), false = close (extend)
   */
  public void setHoodPiston(boolean open) {
    if (open) shooterHoodPiston.set(Value.kReverse);
    else shooterHoodPiston.set(Value.kForward);
  }

  /**
   * @return true = opened (retracted), false = closed (extended)
   */
  public boolean getHoodPiston() {
    if (shooterHoodPiston.get() == Value.kReverse) return true;
    else return false;
  }

  /**
   * @return true = unlocked (retracted), false = locked (extended)
   */
  public boolean getLockPiston() {
    return shooterLockPiston.get();
  }

  /**
   * @param unlock true = unlock (retract), false = lock (extend)
   */
  public void setLockPiston(boolean unlock) {
    shooterLockPiston.set(unlock);
  }

  /**
   * @return PID error, in RPM
   */
  public double getShooterPIDError() {
    // return shooterMotorLeft.getClosedLoopError() * ticksPer100ms;
    return shooterRPM - measuredRPM;
  }

  /**
   * @return measured RPM
   */
  public double getMeasuredRPM() {
    measuredVelocityRaw = shooterMotorLeft.getSelectedSensorVelocity(0);
    measuredRPM = measuredVelocityRaw * ticksPer100ms; // converts ticks per 100ms to RPM
    return measuredRPM;
  }

   /**
   * @return true = power cell in shooter, false = no power cell in shooter
   */
  public boolean getCell() {
    return !input.get();
  }

  /**
   * @return number of power cells shot
   */
  public int getPowerCellsShot() {
    return cellsShot;
  }

  public void setPowerCellsShot(int cells) {
    cellsShot = cells;
  }

  /**
   * Use distance from the target to calculate target RPM. A slope is created between the 
   * two closest values in the table to the parameter distance to calculate the target RPM.
   * This method returns the default RPM if the robot is less than 5 feet from the target, 
   * and the max RPM in the table if the robot is more than 30 feet from the target.
   * @param distance distance from target (as per vision data), in feet
   * @return target RPM for shooter to make it into the target
   */
  public double distanceFromTargetToRPM(double distance) {
    int len = distanceFromTargetToRPMTable.length;
    if(distance < distanceFromTargetToRPMTable[0][0]) return shooterDefaultRPM; /*return distanceFromTargetToRPMTable[0][1];*/
    if(distance > distanceFromTargetToRPMTable[len-1][0]) return distanceFromTargetToRPMTable[len-1][1];
    int leftBound = 0;

    for(int i = len - 1; i >= 0; i--) {
      if(distance > distanceFromTargetToRPMTable[i][0]) {
        leftBound = i;
        i = 0;
      } else if (distance == distanceFromTargetToRPMTable[i][0]) {
        return distanceFromTargetToRPMTable[i][1];
      }
    }

    double lowerRPM = distanceFromTargetToRPMTable[leftBound][1];
    double upperRPM = distanceFromTargetToRPMTable[leftBound + 1][1];
    double dRPMperMeter = (upperRPM - lowerRPM) / 5;
    double targetRPM = ((distance - distanceFromTargetToRPMTable[leftBound][0]) * (dRPMperMeter)) + lowerRPM;
    return targetRPM;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    measuredRPM = getMeasuredRPM();

    // if photo switch is triggered and it was not previously triggered, increase
    // counter for power cells shot and set led strip
    if (getCell() && !prevCellState) {
      cellsShot++;
      led.setBallLights(cellsShot);
    }

    // if voltage target is 0 and current number of power cells shot is not equal to
    // previous number of power cells shot (previous is not 0), set led strip to current n
    if(voltageTarget == 0 && cellsShot != prevCellsShot) led.setBallLights(cellsShot);

    if (voltageTarget == 0) cellsShot = 0;
    if(voltageTarget == 0 && (cellsShot != prevCellsShot)) led.setBallLights(cellsShot);

    if(fastLogging || log.getLogRotation() == log.SHOOTER_CYCLE) {
      updateShooterLog(false);
    }
    
    if(log.getLogRotation() == log.SHOOTER_CYCLE) {

      // read PID coefficients from SmartDashboard
      double ff = SmartDashboard.getNumber("Shooter FF", 0);
      double p = SmartDashboard.getNumber("Shooter P", 0);
      double i = SmartDashboard.getNumber("Shooter I", 0);
      double d = SmartDashboard.getNumber("Shooter D", 0);

      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if(ff != kFF) shooterMotorLeft.config_kF(0, ff, timeoutMs); kFF = ff;
      if(p != kP) shooterMotorLeft.config_kP(0, p, timeoutMs); kP = p;
      if(i != kI) shooterMotorLeft.config_kI(0, i, timeoutMs); kI = i;
      if(d != kD) shooterMotorLeft.config_kD(0, d, timeoutMs); kD = d;

      SmartDashboard.putNumber("Shooter SetPoint RPM", setPoint * ticksPer100ms);
      SmartDashboard.putNumber("Shooter RPM", measuredRPM);
      SmartDashboard.putNumber("Shooter Motor 1 Current", shooterMotorLeft.getSupplyCurrent());
      SmartDashboard.putNumber("Shooter Motor 2 Current", shooterMotorRight.getSupplyCurrent());
      SmartDashboard.putNumber("Shooter PID Error", getShooterPIDError());
      SmartDashboard.putNumber("Shooter Motor 1 PercentOutput", shooterMotorLeft.getMotorOutputPercent());
      SmartDashboard.putNumber("Shooter Motor 2 PercentOutput", shooterMotorRight.getMotorOutputPercent());
      SmartDashboard.putNumber("Shooter Voltage", shooterMotorLeft.getMotorOutputVoltage());
      SmartDashboard.putNumber("Power Cells Shot", cellsShot);
      SmartDashboard.putBoolean("Cell Present", prevCellState);
      SmartDashboard.putBoolean("Hood Open", getHoodPiston());
      SmartDashboard.putBoolean("Lock Locked", getLockPiston());
      // SmartDashboard.putNumber("Shooter Motor 1 Voltage", shooterMotorLeft.getMotorOutputVoltage());
      // SmartDashboard.putNumber("Shooter Motor 2 Voltage", shooterMotorRight.getMotorOutputVoltage());
    }

    prevCellState = getCell();
    prevCellsShot = cellsShot;
  }

  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Write information about shooter to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
	public void updateShooterLog(boolean logWhenDisabled) {
		log.writeLog(logWhenDisabled, "Shooter", "Update Variables",  
      "Left Motor Volt", shooterMotorLeft.getMotorOutputVoltage(), 
      "Right Motor Volt", shooterMotorRight.getMotorOutputVoltage(),
      "Left Motor Amps", shooterMotorLeft.getSupplyCurrent(),
      "Right Motor Amps", shooterMotorRight.getSupplyCurrent(),
      "Measured RPM", measuredRPM,
      "Power Cell", prevCellState
    );
  }

  /**
   * Update TemperatureCheck utility with motors that are and are not overheating.
   */
  public void updateOverheatingMotors() {
    if (shooterMotorLeft.getTemperature() >= temperatureCheck)
      tempCheck.recordOverheatingMotor("ShooterLeft");
    if (shooterMotorRight.getTemperature() >= temperatureCheck)
      tempCheck.recordOverheatingMotor("ShooterRight");

    if (shooterMotorLeft.getTemperature() < temperatureCheck)
      tempCheck.notOverheatingMotor("ShooterLeft");
    if (shooterMotorRight.getTemperature() < temperatureCheck)
      tempCheck.notOverheatingMotor("ShooterRight");
  }
}
