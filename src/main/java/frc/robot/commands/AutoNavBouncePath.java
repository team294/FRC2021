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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoNavBouncePath extends SequentialCommandGroup {
 
  // start robot infront of opponents trench with the intake facing the trench

  public AutoNavBouncePath(DriveTrain driveTrain, FileLog log) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(

      new DriveResetPose(0.762, 2.286, 0.0, driveTrain, log),

      new DriveFollowTrajectory(CoordType.kAbsolute, TrajectoryBounceSToA3.calcTrajectory(log), false, PIDType.kWPILib, driveTrain, log) 
          .andThen(() -> driveTrain.tankDrive(0.0, 0.0, false)), // Might not need to come to a full stop,
                                                                 // TODO test 2 trajectories back to back w/out stopping

      new DriveFollowTrajectory(CoordType.kAbsolute, TrajectoryBounceA3ToA6.calcTrajectory(log), false, PIDType.kWPILib, driveTrain, log) 
          .andThen(() -> driveTrain.tankDrive(0.0, 0.0, false)),

      new DriveFollowTrajectory(CoordType.kAbsolute, TrajectoryBounceA6ToA9.calcTrajectory(log), false, PIDType.kWPILib, driveTrain, log) 
          .andThen(() -> driveTrain.tankDrive(0.0, 0.0, false)),

      new DriveFollowTrajectory(CoordType.kAbsolute, TrajectoryBounceA9ToF.calcTrajectory(log), false, PIDType.kWPILib, driveTrain, log) 
          .andThen(() -> driveTrain.tankDrive(0.0, 0.0, false))
    );
  }
}
