/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FileLog;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotConstants;

import static frc.robot.Constants.IntakeConstants.*;


public class Intake extends SubsystemBase {
  private final BaseMotorController intakeMotor;
  private double intakeCurrent = 0, targetPercentOutput = 0;
  private Timer ledTimer;
  private String ledColor = "Green";

  private final DoubleSolenoid intakePiston = new DoubleSolenoid(pcmIntakePistonOut, pcmIntakePistonIn);
  private FileLog log; 
  private LED led;
 
  public Intake(FileLog log, LED led) {
    this.log = log;
    this.led = led;
    this.ledTimer = new Timer();

    if (RobotConstants.prototypeBot) { 
      intakeMotor = new WPI_VictorSPX(canIntakeMotor);
    }
    else {
      intakeMotor = new WPI_TalonSRX(canIntakeMotor);
    }
    intakeMotor.configFactoryDefault();
    intakeMotor.setInverted(true);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * @param percent percent output (0 to 1)
   */
  public void intakeSetPercentOutput(double percent) {
    intakeMotor.set(ControlMode.PercentOutput, percent);
    targetPercentOutput = percent;
    if (percent == 0) {
      ledTimer.stop();
      ledTimer.reset();
      led.setStrip("Black", 0.5, 1);
    } else if (Math.abs(percent) == IntakeConstants.intakeDefaultPercentOutput && !ledTimer.hasElapsed(0.1)) {
      ledTimer.start();
    }
  }

  /**
   * @return target percent output (0 to 1)
   */
  public double intakeGetPercentOutput() {
    return targetPercentOutput;
  }

  /**
   * @param extend true = extend, false = retract
   */
  public void intakeSetPiston(boolean extend) {
    if (extend) intakePiston.set(Value.kForward);
    else if (!extend) intakePiston.set(Value.kReverse);
  }

  /**
   * @return true = extended, false = retracted
   */
  public boolean intakeGetPiston() {
    if (intakePiston.get() == Value.kForward) return true;
    else return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(log.getLogRotation() == log.INTAKE_CYCLE) {
      if (!RobotConstants.prototypeBot) {
        intakeCurrent = ((WPI_TalonSRX) intakeMotor).getSupplyCurrent();
      } else {
        intakeCurrent = 0;  
      }

      if (ledTimer.advanceIfElapsed(0.1)) {
        led.setStrip(ledColor, 0.5, 1);
        if (ledColor.equals("Green")) ledColor = "Black";
        else ledColor = "Green";
      }

      updateIntakeLog(false);
      
      SmartDashboard.putNumber("Intake % Output", intakeMotor.getMotorOutputPercent());
      SmartDashboard.putNumber("Intake Voltage", intakeMotor.getMotorOutputVoltage()); 
      SmartDashboard.putNumber("Intake Current", intakeCurrent);
      SmartDashboard.putNumber("Intake Percent", intakeMotor.getMotorOutputPercent());
    }
  }

  /**
   * Write information about intake to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
	public void updateIntakeLog(boolean logWhenDisabled) {
		log.writeLog(logWhenDisabled, "Intake", "Update Variables",  
      "Motor Volt", intakeMotor.getMotorOutputVoltage(), 
      "Motor Output %", intakeMotor.getMotorOutputPercent(),
      "Current", intakeCurrent
    );
  }
}
