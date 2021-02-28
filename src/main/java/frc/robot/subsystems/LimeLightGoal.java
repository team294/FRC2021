/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.StringUtil;
import static frc.robot.Constants.LimeLightConstants.*;

public class LimeLightGoal extends LimeLight {
  private Relay flashlight = new Relay(relayFlashlight, Direction.kBoth);
  private LED led;
  private double sweetSpot;
  private boolean setFlashAuto = true;
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
  public LimeLightGoal(FileLog log, LED led, DriveTrain driveTrain) {
    super("Limelight", log);
    this.led = led;
    this.driveTrain = driveTrain;
  }

  /**
   * Distance from the robot to the shooting "sweet spot"
   * @return distance to the "sweet spot", in feet (+ = move towards target)
   */
  public double getSweetSpot() {
    return sweetSpot;
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

  @Override
  public void periodic() {
    super.periodic();
  
    sweetSpot = getDistance() - endDistance;

    if (makePattern() == LED.visionTargetLibrary[15]) {
      led.setPattern(makePattern(), 0.1, 0);
    } else {
      led.setPattern(makePattern(), 0.5, 0);
    }

    if(!isGettingData() && setFlashAuto) {
      setFlashlight(true);
    }
    if (log.getLogRotation() == log.LIMELIGHT_CYCLE) {
      SmartDashboard.putNumber(StringUtil.buildString(limelightName, " new distance"), getDistance()); // distance calculation using vision camera
      SmartDashboard.putNumber(StringUtil.buildString(limelightName, " Actual dist"), (-driveTrain.getAverageDistance()/12)); // distance calculation using drive encoders, used to test accuracy of getDistanceNew()
      SmartDashboard.putNumber(StringUtil.buildString(limelightName, " sweet spot"), sweetSpot);
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
      "Network Table Read Counter", networkTableReadCounter,
      "Snapshot Count", snapshotCount,
      "Dist", getDistance(), "Encoder Dist", (-driveTrain.getAverageDistance()/12)
      );
  }
}
