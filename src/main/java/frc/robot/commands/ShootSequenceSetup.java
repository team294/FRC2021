/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LimeLightGoal;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.FileLog;

public class ShootSequenceSetup extends SequentialCommandGroup {
  
  /**
   * Set the shooter hood position (open or close, lock or unlock).
   * Then set the shooter RPM either with the distance from the target or the default short shot RPM.
   * @param closeHood true = close the hood, false = open the hood
   * @param shooter shooter subsystem
   * @param limeLightGoal limelight camera (subsystem)
   * @param led led strip (subsystem)
   */
  public ShootSequenceSetup(boolean closeHood, Shooter shooter, LimeLightGoal limeLight, LED led, FileLog log) {
    addCommands(
      // If the current distance away from the target is greater than the max distance for 
      // unlocking the hood or vision sees no target, close and lock the hood. 
      // Otherwise, close the hood and leave it unlocked.
      new ConditionalCommand(
        new ShooterHoodPistonSequence(closeHood, true, shooter, log),
        new ShooterHoodPistonSequence(closeHood, false, shooter, log),
        () -> closeHood && (limeLight.getDistance() > LimeLightConstants.unlockedHoodMaxDistance 
          || !limeLight.seesTarget())
      ),
      // If closing the hood, set shooter RPM based on distance.
      // Otherwise, set shooter RPM to the default value for the short shot.
      new ConditionalCommand(
        parallel(
          new FileLogWrite(false, false, "ShootSequence", "Setup", log, "Hood", "Close"),
          new ShooterSetPID(true, false, shooter, limeLight, led, log)
        ),
        parallel(
          new FileLogWrite(false, false, "ShootSequence", "Setup", log, "Hood", "Open"),
          new ShooterSetPID(ShooterConstants.shooterDefaultShortRPM, shooter, led, log)
        ),
        () -> closeHood
      )
    );
  }

  /**
   * Set shooter to setpoint RPM using default values for shooting from the trench
   * or the auto line and set the hood/lock position.
   * @param trench true = shooting from trench, false = shooting from autoline
   * @param shooter shooter subsystem
   * @param led led strip (subsystem)
   */
  public ShootSequenceSetup(boolean trench, Shooter shooter, LED led, FileLog log) {
    addCommands(
      // If shooting from the trench, close the hood, lock it, and set shooter RPM. 
      // Otherwise, close the hood, leave it unlocked, and set shooter RPM.
      new ConditionalCommand(
        sequence(
          new FileLogWrite(false, false, "ShootSequence", "Setup", log, "ShootFrom", "Trench"),
          new ShooterHoodPistonSequence(true, true, shooter, log),
          new ShooterSetPID(false, ShooterConstants.shooterDefaultTrenchRPM, shooter, led, log)
        ),
        sequence(
          new FileLogWrite(false, false, "ShootSequence", "Setup", log, "ShootFrom", "Autoline"),
          new ShooterHoodPistonSequence(true, false, shooter, log),
          new ShooterSetPID(false, ShooterConstants.shooterDefaultRPM, shooter, led, log)
        ),
        () -> trench
      )
    );
  }
}
