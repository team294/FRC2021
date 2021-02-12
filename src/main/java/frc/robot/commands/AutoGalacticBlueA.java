/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoordType;
import frc.robot.commands.DriveFollowTrajectory.PIDType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;

public class AutoGalacticBlueA extends SequentialCommandGroup {
 
  /**
   * Follow the bounce path using trajectories.
   */
  public AutoGalacticBlueA(TrajectoryCache trajectoryCache, DriveTrain driveTrain, FileLog log, Intake intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(

      new DriveResetPose(0.762, 0.762, 0.0, driveTrain, log),
      parallel(
        new DriveFollowTrajectory(CoordType.kAbsolute, trajectoryCache.cache[TrajectoryType.galacticBlueA.value], 
                                  true, true, PIDType.kTalon, driveTrain, log), // follow the trajcectory
        new IntakeSequence(intake, log) // Deploy and run intake
      ) 
    );
  }
}
