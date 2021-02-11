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

public class AutoGalacticRedB extends SequentialCommandGroup {
 
  /**
   * Follow the bounce path using trajectories.
   */
  public AutoGalacticRedB(TrajectoryCache trajectoryCache, DriveTrain driveTrain, FileLog log, Intake intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(

      new DriveResetPose(0.762, 3.81, -45.0, driveTrain, log),
      new IntakePistonSetPosition(true, intake, log), // deploy intake
      new Wait(0.4),
      new IntakeSetPercentOutput(true, intake, log), // start intake
      new DriveFollowTrajectory(CoordType.kAbsolute, trajectoryCache.cache[TrajectoryType.galacticRedB.value], 
                                false, PIDType.kWPILib, driveTrain, log) 
      .andThen(() -> driveTrain.tankDrive(0.0, 0.0, false)) // follow the trajcectory
      
    );
  }
}
