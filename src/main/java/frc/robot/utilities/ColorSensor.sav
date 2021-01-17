/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

/**
 * Class used to read data from the RevRoboticsV3 Color Sensor.
 * @author Audrey Lee
 */
public class ColorSensor {
	
    private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    private final ColorMatch colorMatcher = new ColorMatch();

    public final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    public final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    public final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    public final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    public ColorSensor() {
        colorMatcher.addColorMatch(kBlueTarget);
        colorMatcher.addColorMatch(kGreenTarget);
        colorMatcher.addColorMatch(kRedTarget);
        colorMatcher.addColorMatch(kYellowTarget);
    }

    /**
     * use as getColorNumber().red to get numerical r value in rgb
     * use as getColorNumber().green to get numerical g value in rgb
     * use as getColorNumber().blue to get numerical b value in rgb
     */
    public Color getColorNumber() {
        return colorSensor.getColor();
    }

    /**
     * @return String describing what color is being read based on 
     * target colors described above (blue, green, red, yellow)
     */
    public String getColor() {
        Color detectedColor = getColorNumber();

        String colorString;
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        
        if (match.color == kBlueTarget) {
            colorString = "Blue";
        } else if (match.color == kRedTarget) {
            colorString = "Red";
        } else if (match.color == kGreenTarget) {
            colorString = "Green";
        } else if (match.color == kYellowTarget) {
            colorString = "Yellow";
        } else {
            colorString = "Unknown";
        }
        
        return colorString;
    }

    /**
     * @param check what color (based on target colors) to check for
     * @return whether or not the color "check" is being read
     */
    public boolean isColor(Color check) {
        Color detectedColor = getColorNumber();

        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        
        if (match.color == check) return true;
        return false;
    }

    /**
     * @return IR value being read by the IR sensor
     */
    public double getIR() {
        double IR = colorSensor.getIR();
        SmartDashboard.putNumber("IR", IR);
        return colorSensor.getIR();
    }
    
    /**
     * @return Proximity value being read by the Proximity sensor
     */
    public int getProximity() {
        return colorSensor.getProximity();
    }
    
}
