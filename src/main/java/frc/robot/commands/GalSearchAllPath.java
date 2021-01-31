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

public class GalSearchAllPath extends SequentialCommandGroup {
 
  /**
   * Follow the bounce path using trajectories.
   */
  public GalSearchAllPath(TrajectoryCache trajectoryCache, DriveTrain driveTrain, Intake intake, FileLog log) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(
      deadline(
        new IntakeSequence(intake, log),
        new DriveResetPose(0.382, 3.81, 0.0, driveTrain, log)
      ),

      new DriveFollowTrajectory(CoordType.kAbsolute, trajectoryCache.cache[TrajectoryType.allBall1.value], false, PIDType.kWPILib, 
          driveTrain, log) 
          .andThen(() -> driveTrain.tankDrive(0.0, 0.0, false)), // Might not need to come to a full stop,
                                                                 // TODO test 2 trajectories back to back w/out stopping

      new DriveFollowTrajectory(CoordType.kAbsolute, trajectoryCache.cache[TrajectoryType.allBall2.value], false, PIDType.kWPILib, driveTrain, log) 
          .andThen(() -> driveTrain.tankDrive(0.0, 0.0, false)),

      new DriveFollowTrajectory(CoordType.kAbsolute, trajectoryCache.cache[TrajectoryType.allBall3.value], false, PIDType.kWPILib, driveTrain, log) 
          .andThen(() -> driveTrain.tankDrive(0.0, 0.0, false)),

      new DriveFollowTrajectory(CoordType.kAbsolute, trajectoryCache.cache[TrajectoryType.allBall4.value], false, PIDType.kWPILib, driveTrain, log) 
          .andThen(() -> driveTrain.tankDrive(0.0, 0.0, false)),

      new DriveFollowTrajectory(CoordType.kAbsolute, trajectoryCache.cache[TrajectoryType.allBall5.value], false, PIDType.kWPILib, driveTrain, log) 
          .andThen(() -> driveTrain.tankDrive(0.0, 0.0, false))
    );
  }
}
