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

public class LimeLight extends SubsystemBase implements Loggable {
  private static NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
  private static NetworkTable table = tableInstance.getTable("limelight");
  private NetworkTableEntry tv, tx, ty, ta, tl, pipeline;
  private double targetExists, x, y, area, latency, pipe;
  private Relay flashlight = new Relay(relayFlashlight, Direction.kBoth);
  private FileLog log;
  private LED led;
  private double sweetSpot;
  private Timer snapshotTimer;
  private int snapshotCount = 0;
  private int networkTableReadCounter = 0;
  private boolean setFlashAuto = true;
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
  public LimeLight(FileLog log, LED led, DriveTrain driveTrain) {
    this.log = log;
    this.led = led;
    this.snapshotTimer = new Timer();
    snapshotTimer.reset();
    snapshotTimer.start();
    this.driveTrain = driveTrain;
    tableInstance.startClientTeam(294);

    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tl = table.getEntry("tl");
    pipeline = table.getEntry("pipeline");
    SmartDashboard.putNumber("Pipeline", 0);
  }

  /**
   * @return horizontal (x-axis) angle, in degrees, between camera crosshair and target crosshair
   * + = target is to the left, - = target is to the right
   */
  public double getXOffset() {
    return x;
  }

  /**
   * @return vertical (y-axis) angle, in degrees, between camera crosshair and target crosshair
   * down is negative, up is positive
   */
  public double getYOffset() {
    return y;
  }

  /**
   * @return area of target
   */
  public double getArea() {
    return area;
  }

  /**
   * Distance from the robot to the shooting "sweet spot"
   * @return distance to the "sweet spot", in feet (+ = move towards target)
   */
  public double getSweetSpot() {
    return sweetSpot;
  }

  /**
   * @return latency contribution by pipeline
   */
  public double getLatency() {
    return latency;
  }

  /**
   * Takes into account not being in line with the target.
   * @return distance from camera to target, on the floor, in feet
   */
  public double getDistance() {    //  TODO  this could return a erroneous value if vision misses a frame or is temporarily blocked.  Use avgrging or filtering
    double myDistance = (targetHeight - cameraHeight) / ((Math.tan(Math.toRadians(cameraAngle + y))) * (Math.cos(Math.toRadians(x))));
    return myDistance;
  }

  /**
   * @return current pipeline number
   */
  public double getPipeline() {
    return table.getEntry("getpipe").getDouble(0);
  }

  /**
   * @param pipeNum 0 is vision, 2 is driver cam
   */
  public void setPipe(double pipeNum) {
    pipeline.setDouble(pipeNum);
  }

  /**
   * @param snapshot true is take a snapshot, false is don't take snapshots
   * will increment "snapshotCount" to keep track of how many snapshots have been taken
   * should only be called if canTakeSnapshot() is true
   */
  public void setSnapshot(boolean snapshot) {
    // TODO disable snapshots -- this seems to cause camera lag
    snapshot = false;

    if(snapshot && canTakeSnapshot()) {
      table.getEntry("snapshot").setNumber(1);
      if(table.getEntry("snapshot").getDouble(0)==1) {
        snapshotCount++;
      }
      log.writeLog(false, "LimeLight", "setSnapshot", "Snapshot Count", snapshotCount);

      snapshotTimer.reset();
      snapshotTimer.start();
    }
    else table.getEntry("snapshot").setNumber(0);
  }

  /**
   * Limelight will only save snapshots every 0.5 seconds
   * @return true if enough time has passed since the last snapshot to take another snapshot
   *              in order to prevent overloading the Limelight with snapshot requests
   *              "takeSnapshots" has to be true in preferences (meaning we want to be taking snapshots)
   *         false if we shouldn't take a snapshot (either due to delay or not wanting to take snapshots)
   */
  public boolean canTakeSnapshot() {
    return snapshotTimer.hasElapsed(0.5) && takeSnapshots;
  }

  /**
   * reset number of snapshots taken (as if we are clearing our cache of snapshots)
   * This will NOT actually delete the snapshots (we don't have that capability in code)
   */
  public void resetSnapshotCount() {
    snapshotCount = 0;
  }

  /**
   * @param on true = turn on the flashlight, false = turns off the flashlight
   */
  public void setFlashlight(boolean on) {
    if(on) {
      flashlight.set(Value.kForward);
    } else {
      flashlight.set(Value.kReverse);
    }
  }

  /**
   * @param auto true = automatically turn on the flashlight when we lose vision
   * false = allow driver to turn flashlight on/off
   */
  public void setFlashlightAuto(boolean auto) {
    setFlashAuto = auto;
  }

  /**
   * Choose which LED pattern to display, based on the x offset from camera.
   * @return Color array of the pattern
   */
  public Color[] makePattern() {
    Color[] myPattern = new Color[16];
    int patternFormula = (int)(-x + 7);
    if (patternFormula < 0) {
      patternFormula = 0;
    } else if (patternFormula > 14) {
      patternFormula = 14;
    }

    myPattern = LED.visionTargetLibrary[patternFormula];
    if (!seesTarget()) {
      myPattern = LED.visionTargetLibrary[15];
    }
    return myPattern;
  }
//TODO increase limelight logging and edit seesTarget to check last three periodic for target
  /**
   * @return true when limelight sees a target, false when not seeing a target
   */
  public boolean seesTarget() {
    return (targetExists==1);
  }

  /**
   * @return true when limelight is connected & reading data
   * false when limelight is disconnected or not reading any data   TODO when would x be 1064??
   */
  public boolean isGettingData() {
    return (x != (1000 * LimeLightConstants.angleMultiplier) && y != 1000);
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

    sweetSpot = getDistance() - endDistance;

    if (makePattern() == LED.visionTargetLibrary[15]) {
      led.setPattern(makePattern(), 0.1, 0);
    } else {
      led.setPattern(makePattern(), 0.5, 0);
    }

    if(!isGettingData() && setFlashAuto) {
      setFlashlight(true);
    }
    
    if (fastLogging || log.getLogRotation() == log.LIMELIGHT_CYCLE) {
      updateLimeLightLog(false);
    }
    
    if (log.getLogRotation() == log.LIMELIGHT_CYCLE) {


      if(!isGettingData()) {
        RobotPreferences.recordStickyFaults("LimeLight", log);
      }

      // Invert X on SmartDashboard, since bars on SmartDashboard always go from - (left) to + (right)
      SmartDashboard.putNumber("LimeLight x", -x);
      SmartDashboard.putNumber("LimeLight y", y);
      SmartDashboard.putBoolean("Limelight Sees Target", seesTarget());
      //SmartDashboard.putNumber("Limelight dist", getDistance()); // distance assuming we are in line with the target
      SmartDashboard.putNumber("Limelight new distance", getDistance()); // distance calculation using vision camera
      SmartDashboard.putNumber("Limelight Actual dist", (-driveTrain.getAverageDistance()/12)); // distance calculation using drive encoders, used to test accuracy of getDistanceNew()
      SmartDashboard.putNumber("Limelight sweet spot", sweetSpot);
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

  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
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
