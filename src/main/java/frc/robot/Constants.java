/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class RobotConstants {
        // Global constants go here
        public static final double temperatureCheck = 40; // in celsius
        public static final double pidErrorTolerance = 200; // in RPM

        // Next row is a DEFAULT VALUE. Change this value in RobotPrefrences for each
        // robot, not in this code!
        public static boolean prototypeBot = false; // true = proto robot, false = competition robot
    }

    public static final class ShooterConstants {
        public static final int canShooterMotorRight = 31; // 30 on competition bot
        public static final int canShooterMotorLeft = 30; // 31 on competition bot
        public static final int pcmShooterHoodPistonIn = 2; // open hood (retract)
        public static final int pcmShooterHoodPistonOut = 3; // close hood (extend)
        public static final int pcmShooterLockPiston = 6; // lock and unlock hood angle
        public static final double shooterDefaultRPM = 2800;
        public static final double shooterDefaultTrenchRPM = 3000;
        public static final double shooterDefaultShortRPM = 1450; // originally 1400 in competition
        //public static final double voltageCheck = 7.5; // voltage the shooter will reach if power cell is shot (for
                                                       // counting power cells)
        public static final int dioPowerCell = 9;
        // public static final double voltageCheck = 7.5; // voltage the shooter will reach if power cell is shot (for counting power cells)
        // public static final double currentCheck = 60; // voltage the shooter will reach if power cell is shot (for counting power cells)
        public static final double hopperPercentCheck = 0.3; // percent output hopper will reach once it is running (for counting power cells)
        
        public static final int[][] distanceFromTargetToRPMTable = {{5,1450},{10,2250},{15,2450},{20,2500},{25,2550}};
        // Original table: {{4,1400*1.1},{5,1500*1.1},{10,2500*1.1},{15,2900*1.1},{20,2900*1.1},{25,3100*1.1},{30,3200*1.1}};
        // TODO figure out max distance of robot from target so table includes all necessary values

        public static final double maxSecondsToShoot3balls = 5.0; // max time to wait while shooting 3 balls. use this in commands to timeout
    }

    public static final class FeederConstants {
        public static final int canFeederMotor = 40;
        public static final double feederDefaultRPM = 2000*1.5;
    }

    public static final class IntakeConstants {
        public static final int canIntakeMotor = 50;
        public static final int pcmIntakePistonIn = 0;
        public static final int pcmIntakePistonOut = 1;
        public static final double intakeDefaultPercentOutput = 0.8;
        public static final double intakeShootingPercentOutput = 0.7;
    }

    public static final class HopperConstants {
        public static final int canHopperMotor = 51;
        public static final double hopperDefaultPercentOutput = 0.8;
    }

    public static final class ClimbConstants {
        public static final int canClimbMotorLeft = 46;
        public static final int canClimbMotorRight = 45;
        public static final int pcmClimbPistonsIn = 4;
        public static final int pcmClimbPistonsOut = 5;
        public static final int pcmClimbPistonLock = 7;
        public static final double ticksPerInch = 2048 * 54 / 2 / Math.PI; // ticksPerRotation * gearRatio(54:1) / pi / radius
        public static final double positionTolerance = 1; // inches from the target position to stop applying power to the motor
        public static final double maxHeight = 33;
        public static final double liftHeight = 5; // height to bring climb down to when lifting robot, in inches
        public static final double latchHeight = 25; // default height to bring climb up to latch on, in inches
        public static final double latchExtensionHeight = 28; // height slightly above default height to bring climb up to latch on, in inches
        public static final double defaultVelocity = -6;
    }

    public static final class LimeLightConstants {
        public static final double angleMultiplier = 1.064;
        public static final double offset = 1.33; // in feet
        public static final double cameraHeight = 1.6666667; // in feet, height from floor to lens of mounted camera, 2.104 on protobot
        public static final double targetHeight = 7.0; // in feet, height to middle of crosshair on target
        public static final double cameraAngle = 22.5; // in degrees 26.5 measured but 28 works better?, 14 on proto
        public static final double endDistance = 18; // distance of the "sweet spot", in feet
        public static final double unlockedHoodMaxDistance = 13.8; // greatest feet away from target that hood needs to be unlocked to make shot
        
        // Flashlight constants
        public static final int relayFlashlight = 0;

        // *******************************
        // The constants below are DEFAULT VALUES. Change these value in RobotPrefrences
        // for each robot, not in this code!
        // *******************************
        public static boolean takeSnapshots = true;
    }

    /**
     * Options to select driving coordinates.
     */
    public enum CoordType {
        kRelative(0),
        kAbsolute(1),
        kAbsoluteResetPose(2);
    
        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        CoordType(int value) { this.value = value; }
    }
    /**
     * Options to select path for galactic search.
     */
    public enum SearchType {
        kRedA(0),
        kRedB(1),
        kBlueA(2),
        kBlueB(3),
        kNull(4);

        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        SearchType(int value) { this.value = value; }
    }
    /**
     * Options to select driving target types.
     */
    public enum TargetType {
        kRelative(0),
        kAbsolute(1),
        kVision(2);
    
        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        TargetType(int value) { this.value = value; }
    }

    /**
     * Options to select driving stopping types.
     */
    public enum StopType {
        kNoStop(0),
        kCoast(1),
        kBrake(2);
    
        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        StopType(int value) { this.value = value; }
    }

    public static final class DriveConstants {

        // *******************************
        // The constants below apply to all robots and are not in RobotPreferences
        // *******************************

        public static final int canLeftDriveMotor1 = 10;
        public static final int canLeftDriveMotor2 = 11;

        public static final int canRightDriveMotor1 = 20;
        public static final int canRightDriveMotor2 = 21;

        public static final double compensationVoltage = 12.0; // voltage compensation on drive motors
        public static final double MAX_VOLTAGE_IN_TRAJECTORY = 10.0;

        // suggested from tutorial
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.70;

        // *******************************
        // The constants below are DEFAULT VALUES. Change these value in RobotPrefrences
        // for each robot, not in this code!
        // *******************************

        public static double ticksPerInch = 1210.0; // TODO Practice bot = 830.8, 1103.9 on competition bot (2020), 1210.0 (was 1195.0) on competition bot in parking lot (2021)

        // public static double wheelDiameterInches = 6.1; //TODO set wheel diameter
        // with actual robot values
        // public static double wheelCircumferenceInches = wheelDiameterInches *
        // Math.PI;
        // public static double encoderTicksPerRevolution = 2048 * 9.47; // Gear ratio =
        // 9.47 on competition bot
        // public static final double kEncoderDistanceInchesPerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        // (wheelDiameterInches * Math.PI) / (double) encoderTicksPerRevolution;
        // 1/ticksPerInch;

        // turnGyro constants
        public static double kMaxAngularVelocity = 1125; // degrees per second TODO calculate on actual 2020 robot
        public static double kMaxAngularAcceleration = 200; // degrees per second per second 200 on competition bot
        public static double kVAngular = 0.00108; // 0.00108 calibrated on competition bot
        public static double kAAngular = 0.00015;  // 0.00015 calibrated on competition bot
        public static double kSAngular = 0.01;   // 0.056 calibrated on competition bot.  Reduced to 0.01 arbitrarily to reduce oscillation.
        public static double kPAngular = 0.001;   // 0.001 calibrated on competition bot (higher values get unstable)
        public static double kDAngular = 0;
        public static double kIAngular = 0.015;    // 0.010 calibrated on competition bot (good for +/-2 degree accuracy, should not oscillate for +/-1 degree).  Increased to 0.015 in parking lot.
        public static double tLagAngular = 0.020;          // Lag time to start/stop turning, or just one cycle forcast through scheduler
        public static final double maxSecondsForTurnGyro = 2.0; // max time to wait for turn gyro. use this in commands to timeout

        public static double wheelInchesToGyroDegrees = 4.205; // converts from inches traveled by the wheels when spinning in place to degrees turned

        // DriveStraight constants
        public static double kMaxSpeedMetersPerSecond = 5.22; // 5.0 on practice bot, 5.22 on competition bot
        public static double kMaxAccelerationMetersPerSecondSquared = 3.8; // 3.8 on practice bot, 3.8 on competition bot
        public static double kVLinear = 0.187; // 0.148 on practice bot, 0.187 on competition bot
        public static double kALinear = 0.025; // 0.025 on practice bot, 0.0184 on competition bot (competition cal=0.0184)
        public static double kSLinear = 0.024; // 0.022 on practice bot, 0.024 on competition bot
        public static double kPLinear = 0.280; // 0.100 on practice bot, 0.100 on competition bot (tuned for drive straight?) -- 1/20/21 changed to 0.28 on competition bot for trajectory following
        public static double kILinear = 0; // 0.0 on both bot
        public static double kDLinear = 0; // 0.0 on both bot
        public static double kAngLinear = 0.030; // 0.030 on both bots

        // Trajectory generation constants
        public static double kS = kSLinear * compensationVoltage; 
        public static double kV = kVLinear * compensationVoltage; 
        public static double kA = kALinear * compensationVoltage; 

        public static double TRACK_WIDTH = 0.71;   // 25.35in on practice bot, 24.93in on competition bot -- 2/14/21 changed to 0.71m on competition bot for tracjectory following

        public static void updateDerivedConstants() {
            kS = kSLinear * compensationVoltage; 
            kV = kVLinear * compensationVoltage; 
            kA = kALinear * compensationVoltage; 
        }
    }

    public static final class OIConstants {
        public static final int usbXboxController = 0;
        public static final int usbLeftJoystick = 1;
        public static final int usbRightJoystick = 2;
        public static final int usbCoPanel = 3;
    }
}
