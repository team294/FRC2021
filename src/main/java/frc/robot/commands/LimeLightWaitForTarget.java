/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLight;

public class LimeLightWaitForTarget extends CommandBase {
  private LimeLight limeLight;
  
  /**
   * Finishes when the limeLight sees the target
   * @param limeLight the limelight
   */
  public LimeLightWaitForTarget(LimeLight limeLight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLight = limeLight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limeLight.enableFastLogging(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limeLight.enableFastLogging(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return limeLight.seesTarget();
  }
}
