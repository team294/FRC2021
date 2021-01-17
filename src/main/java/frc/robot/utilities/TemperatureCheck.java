/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Utility to see what Falcon motors are overheating.
 */
public class TemperatureCheck {
    private String overheatingMotors = "";
    private boolean motorOverheating = false;

    /**
     * Method to add overheating motor to string.
     * @param motor motor name
     */
    public void recordOverheatingMotor(String motor) {
        if (overheatingMotors.indexOf(motor) == -1) {
            if (overheatingMotors.length() != 0) {
                overheatingMotors += ",";
            }
            overheatingMotors += motor;
        }
        if (!motorOverheating) motorOverheating = true;
    }

    /**
     * Method to remove motor from string of overheating motors.
     * @param motor motor name
     */
    public void notOverheatingMotor(String motor) {
        if (overheatingMotors.indexOf(motor) != -1) {
            overheatingMotors = overheatingMotors.replaceAll("," + motor, "");
            overheatingMotors = overheatingMotors.replaceAll(motor + ",", "");
        }
        if (overheatingMotors.length() == 0) motorOverheating = false;
    }

    /**
     * Set string to "" and boolean to false.
     */
    public void resetOverheatingMotors() {
        overheatingMotors = "";
        motorOverheating = false;
    }

    /**
     * Display data on SmartDashboard.
     */
    public void displayOverheatingMotors() {
        SmartDashboard.putString("Overheating Motors", overheatingMotors);
        SmartDashboard.putBoolean("Overheating", motorOverheating);
    }
}
