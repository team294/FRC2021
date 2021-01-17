/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TargetType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class VisionAssistSequence extends SequentialCommandGroup {
  /**
   * Creates a new VisionAssistSequence.
   */

  public VisionAssistSequence(DriveTrain driveTrain, LimeLight limeLight, FileLog log, Shooter shooter, Feeder feeder, LED led, Hopper hopper, Intake intake) {
    super();
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(

    new FileLogWrite(false, false, "VisionAssistSequence", "Init", log),
    
    deadline(
      new DriveTurnGyro(TargetType.kVision, 0, 450, 200, 4, driveTrain, limeLight, log), // turn towards the general target
      new ShootSequenceSetup(true, shooter, limeLight, led, log)
    ),

    //  new DriveStraight(limeLight.getSweetSpot(), TargetType.kVision, 0, 10, 10, true, driveTrain, limeLight, log),
    new DriveStraight(true, TargetType.kVision, 0, 2.61, 3.8, true, driveTrain, limeLight, log),
    
    new DriveTurnGyro(TargetType.kVision, 0, 450, 200, 1, driveTrain, limeLight, log),

    new ShootSequence(true, shooter, feeder, hopper, intake, limeLight, led, log) // turn shooter on until codriver turns it off manually

    );
    
  }
}
