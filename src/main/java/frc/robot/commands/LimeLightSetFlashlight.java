/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLight;
import frc.robot.utilities.FileLog;

public class LimeLightSetFlashlight extends CommandBase {
  /**
   * Creates a new LimeLightFlashlightSet.
   */
  private LimeLight limeLight;
  private FileLog log;
  private boolean on;
  private boolean end;

  
  public LimeLightSetFlashlight(boolean on, boolean end, LimeLight limeLight, FileLog log) {
    this.limeLight = limeLight;
    this.log = log;
    this.on = on;
    this.end = end;
    addRequirements(limeLight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "LimeLightSetFlashlight", "Init", "LightState", (on) ? "On" : "Off");
    limeLight.setFlashlight(on);
    limeLight.setFlashlightAuto(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!end) limeLight.setFlashlight(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
