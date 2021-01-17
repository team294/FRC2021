/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class LEDSetVisionPattern extends CommandBase {
  private LED led;
  private FileLog log;
  private int rowNumber;
  private double intensity;
 
  /**
   * Send a pattern to the LED strip, with parameter intensity.
   * @param rowNumber row in the patternLibrary
   * @param intensity percent intensity (0 to 1)
   * @param led led strip (subsystem)
   */
  public LEDSetVisionPattern(int rowNumber, double intensity, LED led, FileLog log) {
    this.led = led;
    this.log = log;
    this.rowNumber = rowNumber;
    this.intensity = intensity;
    addRequirements(led);
  }

  /**
   * Send a pattern to the LED strip, with default 0.5 intensity.
   * @param rowNumber row in the patternLibrary
   * @param led led strip (subsystem)
   */
  public LEDSetVisionPattern(int rowNumber, LED led, FileLog log) {
    this.led = led;
    this.log = log;
    this.rowNumber = rowNumber;
    this.intensity = 0.5;
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.setPattern(LED.visionTargetLibrary[rowNumber], intensity, 1);
    log.writeLog(false, "LEDSetVisionPattern", "Init", "Row #", rowNumber);
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
