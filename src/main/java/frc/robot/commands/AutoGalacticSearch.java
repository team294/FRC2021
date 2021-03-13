/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.SearchType;
import frc.robot.Constants.StopType;
import frc.robot.commands.DriveFollowTrajectory.PIDType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;

public class AutoGalacticSearch extends SequentialCommandGroup {
 
  

  /**
   * Follow the bounce path using trajectories.
   */
  public AutoGalacticSearch(SearchType path, TrajectoryCache trajectoryCache, DriveTrain driveTrain, FileLog log, Intake intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());

    if(path == SearchType.kRedA){ //Path Red A
      addCommands(
        new IntakePistonSetPosition(true, intake, log),
        new Wait(0.4),
        new IntakeSetPercentOutput(true, intake, log),
        new DriveFollowTrajectory(CoordType.kAbsoluteResetPose, StopType.kBrake, trajectoryCache.cache[TrajectoryType.galacticRedA.value], 
                                    true, PIDType.kTalon, driveTrain, log) // follow the trajcectory
      );
    }

    if(path == SearchType.kRedB){ //Path Red B
      addCommands(
        new IntakePistonSetPosition(true, intake, log), // deploy intake
        new Wait(0.4),
        new IntakeSetPercentOutput(true, intake, log), // start intake
        new DriveFollowTrajectory(CoordType.kAbsoluteResetPose, StopType.kBrake, trajectoryCache.cache[TrajectoryType.galacticRedB.value], 
                                  true, PIDType.kWPILib, driveTrain, log) // follow the trajcectory
      );
    }

    if (path == SearchType.kBlueA){ //Path Blue A
      addCommands(    
        parallel(
            new DriveFollowTrajectory(CoordType.kAbsoluteResetPose, StopType.kBrake, trajectoryCache.cache[TrajectoryType.galacticBlueA.value], 
                                      true,  PIDType.kTalon, driveTrain, log), // follow the trajcectory
            new IntakeSequence(intake, log) // Deploy and run intake
        )
      );
    }

    if(path == SearchType.kBlueB){
      addCommands(
      parallel(
        new DriveFollowTrajectory(CoordType.kAbsoluteResetPose, StopType.kBrake, trajectoryCache.cache[TrajectoryType.galacticBlueB.value], 
                                  true, PIDType.kTalon, driveTrain, log), // follow the trajcectory
        new IntakeSequence(intake, log) // Deploy and run intake
      ) 
      );
    }

    if(path == SearchType.kNull){
      addCommands(
        parallel(
          new Wait(1)
        ) 
        );
    }

  }
}
