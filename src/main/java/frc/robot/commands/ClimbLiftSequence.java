/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.LED;
import frc.robot.utilities.FileLog;

public class ClimbLiftSequence extends SequentialCommandGroup {
  /**
   * Run rainbow LEDs and climb arms lower to lifting height using default target velocity.
   * @param climb climb subsystem
   * @param led led strip (subsystem)
   * @param log filelog utility
   */
  public ClimbLiftSequence(Climb climb, LED led, FileLog log) {
    addCommands(
      new ClimbEnableLimits(false, climb, log),
      parallel(
        new LEDSetPattern(LED.rainbowLibrary, 1, 0.25, led, log),
        new ClimbSetVelocity(true, ClimbConstants.liftHeight, climb, log)
      )
    );
  }
}
