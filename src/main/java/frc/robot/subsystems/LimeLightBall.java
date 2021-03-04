/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.StringUtil;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;
import edu.wpi.first.networktables.NetworkTableEntry;


public class LimeLightBall extends LimeLight {

  /*
   * Limelight settings: TODO Update for Ball Camera!!!
   * ~~input~~ Exposure: 2 Black level offset: 0 red balance:
   * 1200 blue balance: 1975 ~~thresholding~~ hue: 60 - 95 saturation: 150 - 255
   * value: 170 - 255 erosion steps: 0 dilation steps: 0 ~~countour filtering~~
   * area % image: 0.0163 - 100 fullness % of blue rectangle: 10 - 100 w/h ratio:
   * 0 - 20 direction filter : none smart specle detection: 0 target grouping:
   * single target intersection filter: none ~~output~~ targeting region: center
   * send raw corners? : nah send raw contours: no crosshair mode: Single
   * crosshair x: 0 y: 0 ~~3d experimental~~ no changes
   */
  protected NetworkTableEntry tx0, ty0, ta0, tx1, ty1, ta1, tx2, ty2, ta2;
 
  
  public LimeLightBall(FileLog log) {
    super("limelight-ball", log);
    ta0 = table.getEntry("ta0");
    ta1 = table.getEntry("ta1");
    ta2 = table.getEntry("ta2");
    tx0 = table.getEntry("tx0");
    tx1 = table.getEntry("tx1");
    tx2 = table.getEntry("tx2");
    ty0 = table.getEntry("ty0");
    ty1 = table.getEntry("ty1");
    ty2 = table.getEntry("ty2");
  }

  
  /**
   * Takes into account not being in line with the target.
   * @return distance from camera to target, on the floor, in feet
   */
  public TrajectoryType getGalacticPath() {    
    // TODO Update this method to return a SearchType instead of a TrajectoryType.
    // TODO Remove TrajectoryType "galacticNull" from TrajectoryCache.
    // TODO Add SearchType = "NotFound" (in Constants)
    // TODO Add SearchType = "NotFound" in the main "if" statement in the AutoGalacticSearch command (just do a wait command)

    TrajectoryType myTrajectory = TrajectoryType.galacticNull;

    if (!seesTarget()){
      myTrajectory = TrajectoryType.galacticNull;
    } else if(area > 0.25 && x < 0){
      myTrajectory = TrajectoryType.galacticRedB;
    } else if (area > 0.25 &&  x > 0) {
      myTrajectory = TrajectoryType.galacticRedA;
    } else if (x < 0){
      myTrajectory = TrajectoryType.galacticBlueB;
    } else {
      myTrajectory = TrajectoryType.galacticBlueA;
    }

     return myTrajectory;
  }


  @Override
  public void periodic() {
    super.periodic();

    if (log.getLogRotation() == log.LIMELIGHT_CYCLE) {
      SmartDashboard.putNumber(StringUtil.buildString(limelightName, " Galactic Trajectory Chosen"), getGalacticPath().value); // distance calculation using vision camera
    }
  }

  @Override
  public void readData(){
    //double targetExistsNew, xNew, yNew, areaNew, latencyNew;
    double x0, x1, x2, y0, y1, y2, a0, a1, a2;
    double x0Old, y0Old;
    x0 = tx0.getDouble(1000.0);
    y0 = ty0.getDouble(1000.0);
    networkTableReadCounter = 0;
  
    // Check if the Limelight updated the NetworkTable while we were reading values, to ensure that all
    // of the data (targetExists, X, Y, etc) are from the same vision frame.
    do {
      //these two below lines are the first and last values read- are tested in the while loop
      //to see if they have changed. If they change, then we read the data again.
      x0Old = x0;
      y0Old = y0;

      x0 = tx0.getDouble(1000.0);
      x1 = tx1.getDouble(1000.0);
      x2 = tx2.getDouble(1000.0);
      
      a0 = ta0.getDouble(1000.0);
      a1 = ta1.getDouble(1000.0);
      a2 = ta2.getDouble(1000.0);

      targetExists = tv.getDouble(1000.0);
      latency = tl.getDouble(1000.0);

      y1 = ty1.getDouble(1000.0);
      y2 = ty2.getDouble(1000.0);
      y0 = ty0.getDouble(1000.0);

      networkTableReadCounter++;
    } while(networkTableReadCounter<= 5 && (x0Old != x0 || y0Old != y0));

    //finding the biggest ball in sight
    if(a0 > a1 && a0 > a2){
      area = a0;
      x = x0;
      y = y0;
    } else if (a1 > a2) {
      area = a1;
      x = x1;
      y = y1;
    } else {
      area = a2;
      x = x2;
      y = y2;
    }

  }


  /**
   * Write information about limelight to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
  public void updateLimeLightLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, limelightName, "Update Variables", 
      "Target Valid", seesTarget(),
      "Center Offset X", x, 
      "Center Offset Y", y,
      "Target Area", area,
      "Latency", latency,
      "Galactic Trajectory Chosen", getGalacticPath().value,
      "Network Table Read Counter", networkTableReadCounter,
      "Snapshot Count", snapshotCount
      );
  }
}
