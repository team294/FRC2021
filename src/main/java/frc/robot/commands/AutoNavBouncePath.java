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

public class AutoNavBouncePath extends SequentialCommandGroup {
 
  /**
   * Follow the bounce path using trajectories.
   */
  public AutoNavBouncePath(TrajectoryCache trajectoryCache, DriveTrain driveTrain, FileLog log) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(
      new DriveFollowTrajectory(CoordType.kAbsoluteResetPose, trajectoryCache.cache[TrajectoryType.bounceSToA3.value], true, true, PIDType.kTalon, driveTrain, log),

      new DriveFollowTrajectory(CoordType.kAbsolute, trajectoryCache.cache[TrajectoryType.bounceA3ToA6.value], true, true, PIDType.kTalon, driveTrain, log),

      new DriveFollowTrajectory(CoordType.kAbsolute, trajectoryCache.cache[TrajectoryType.bounceA6ToA9.value], true, true, PIDType.kTalon, driveTrain, log),

      new DriveFollowTrajectory(CoordType.kAbsolute, trajectoryCache.cache[TrajectoryType.bounceA9ToF.value], true, true, PIDType.kTalon, driveTrain, log)
    );
  }
}
