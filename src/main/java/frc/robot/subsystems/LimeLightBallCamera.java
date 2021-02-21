/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.RobotPreferences;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import static frc.robot.Constants.LimeLightConstants.*;

public class LimeLightBallCamera extends LimeLight {
  private static NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
  private static NetworkTable table = tableInstance.getTable("limelight");
  private NetworkTableEntry tv, tx, ty, ta, tl, pipeline;
  private double targetExists, x, y, area, latency, pipe;
  private FileLog log;
  private int snapshotCount = 0;
  private int networkTableReadCounter = 0;
  private boolean fastLogging = false;
  private DriveTrain driveTrain; // for testing distance calculation TODO take out once dist calc is finished

  /*
   * Limelight settings: ~~input~~ Exposure: 2 Black level offset: 0 red balance:
   * 1200 blue balance: 1975 ~~thresholding~~ hue: 60 - 95 saturation: 150 - 255
   * value: 170 - 255 erosion steps: 0 dilation steps: 0 ~~countour filtering~~
   * area % image: 0.0163 - 100 fullness % of blue rectangle: 10 - 100 w/h ratio:
   * 0 - 20 direction filter : none smart specle detection: 0 target grouping:
   * single target intersection filter: none ~~output~~ targeting region: center
   * send raw corners? : nah send raw contours: no crosshair mode: Single
   * crosshair x: 0 y: 0 ~~3d experimental~~ no changes
   */
  
  public LimeLightBallCamera(FileLog log, LED led, DriveTrain driveTrain) {
    super(log, led, driveTrain);
    SmartDashboard.putNumber("Pipeline Ball Camera", 0);
  }

  
  /**
   * Takes into account not being in line with the target.
   * @return distance from camera to target, on the floor, in feet
   */
  public double getDistanceFromBall() {    //  TODO calculate distance from closest yellow ball
    double myDistance = 0;
     return myDistance;
  }


  @Override
  public void periodic() {
    // read values periodically
    double targetExistsNew, xNew, yNew, areaNew, latencyNew; 
    targetExistsNew = tv.getDouble(1000.0);
    xNew = -tx.getDouble(1000.0) * LimeLightConstants.angleMultiplier;
    yNew = ty.getDouble(1000.0);
    areaNew = ta.getDouble(1000.0);
    latencyNew = tl.getDouble(1000.0);
    networkTableReadCounter = 0;
  
    // Check if the Limelight updated the NetworkTable while we were reading values, to ensure that all
    // of the data (targetExists, X, Y, etc) are from the same vision frame.
    do {
      targetExists = targetExistsNew;
      x = xNew;
      y = yNew;
      area = areaNew;
      latency = latencyNew;

      targetExistsNew = tv.getDouble(1000.0);
      xNew = -tx.getDouble(1000.0) * LimeLightConstants.angleMultiplier;
      yNew = ty.getDouble(1000.0);
      areaNew = ta.getDouble(1000.0);
      latencyNew = tl.getDouble(1000.0);
      networkTableReadCounter++;
    } while(networkTableReadCounter<= 5 && (targetExistsNew != targetExists || xNew != x || yNew != y
    || areaNew != area || latencyNew != latency));

   
    
    if (fastLogging || log.getLogRotation() == log.LIMELIGHT_CYCLE) {
      updateLimeLightLog(false);
    }
    
    if (log.getLogRotation() == log.LIMELIGHT_CYCLE) {


      if(!isGettingData()) {
        RobotPreferences.recordStickyFaults("LimeLight", log);
      }

      // Invert X on SmartDashboard, since bars on SmartDashboard always go from - (left) to + (right)
      SmartDashboard.putNumber("LimeLight x Ball", -x);
      SmartDashboard.putNumber("LimeLight y Ball", y);
      SmartDashboard.putBoolean("Limelight Sees Ball", seesTarget());
      //SmartDashboard.putNumber("Limelight dist", getDistance()); // distance assuming we are in line with the target
      SmartDashboard.putNumber("Limelight new distance", getDistance()); // distance calculation using vision camera
      SmartDashboard.putNumber("Limelight Actual dist", (-driveTrain.getAverageDistance()/12)); // distance calculation using drive encoders, used to test accuracy of getDistanceNew()
      
      SmartDashboard.putBoolean("Limelight Updating", isGettingData());
      SmartDashboard.putNumber("Limelight Latency", getLatency());
      SmartDashboard.putNumber("Limelight Snapshot Count", snapshotCount);
      
      pipe = SmartDashboard.getNumber("Pipeline", 0); // default is vision pipeline

      if (getPipeline() != pipe) {
        log.writeLogEcho(false, "LimeLight", "Pipeline change", "Pipeline", pipe);
        setPipe(pipe);
      }
    }
  }


  /**
   * Write information about limelight to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
  public void updateLimeLightLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, "LimeLight", "Update Variables", 
      "Target Valid", seesTarget(),
      "Center Offset X", x, 
      "Center Offset Y", y,
      "Target Area", area,
      "Latency", latency,
      "Network Table Read Counter", networkTableReadCounter,
      "Snapshot Count", snapshotCount,
      "Dist", getDistance(), "Encoder Dist", (-driveTrain.getAverageDistance()/12)
      );
  }
}
