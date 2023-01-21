// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

import org.carlmontrobotics.lib199.swerve.SwerveConfig;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double g = 9.81; //meters per second squared

    public static final class Drivetrain {
        public static final double wheelBase = Units.inchesToMeters(28.75);
        public static final double trackWidth = Units.inchesToMeters(19.75);
        // "swerveRadius" is the distance from the center of the robot to one of the modules
        public static final double swerveRadius = Math.sqrt(Math.pow(wheelBase / 2, 2) + Math.pow(trackWidth / 2, 2));
        // The gearing reduction from the drive motor controller to the wheels
        // Gearing for the Swerve Modules is 6.75 : 1
        public static final double driveGearing = 6.75;

        public static final double driveModifier = 1;
        public static final double wheelDiameterMeters = Units.inchesToMeters(4.0) * 7.36/7.65 /* empirical correction */;
        public static final double mu = 0.5; /* 70/83.2;  */

        public static final double NEOFreeSpeed = 5676 * (2 * Math.PI) / 60;    // radians/s
        // Angular speed to translational speed --> v = omega * r / gearing
        public static final double maxSpeed = NEOFreeSpeed * (wheelDiameterMeters / 2.0) / driveGearing * 0.7;
        public static final double maxForward = maxSpeed;
        public static final double maxStrafe = maxSpeed;
        // maxRCW is the angular velocity of the robot.
        // Calculated by looking at one of the motors and treating it as a point mass moving around in a circle.
        // Tangential speed of this point mass is maxSpeed and the radius of the circle is sqrt((wheelBase/2)^2 + (trackWidth/2)^2)
        // Angular velocity = Tangential speed / radius
        public static final double maxRCW = maxSpeed / swerveRadius;

        public static final boolean[] reversed = {false, false, false, false};
        // public static final boolean[] reversed = {true, true, true, true};
        // Determine correct turnZero constants (FL, FR, BL, BR)
        public static final double[] turnZero = {-6.9433, 180-4.2188, -72.9492, 180-4.9218};

        // kP, kI, and kD constants for turn motor controllers in the order of front-left, front-right, back-left, back-right.
        // Determine correct turn PID constants
        public static final double[] turnkP = {0.00374, 0.00374, 0.00374, 0.00374};
        public static final double[] turnkI = {0, 0, 0, 0};
        public static final double[] turnkD = {0, 0, 0, 0};
        public static final double[] turnkS = {0.2, 0.2, 0.2, 0.2};
        // V = kS + kV * v + kA * a
        // 12 = 0.2 + 0.00463 * v
        // v = (12 - 0.2) / 0.00463 = 2548.596 degrees/s
        public static final double[] turnkV = {0.00463, 0.00463, 0.00463, 0.00463};
        public static final double[] turnkA = {0.000115, 0.000115, 0.000115, 0.000115};

        // kP is an average of the forward and backward kP values
        // Forward: 1.72, 1.71, 1.92, 1.94
        // Backward: 1.92, 1.92, 2.11, 1.89
        public static final double[] drivekP = {1.82, 1.815, 2.015, 1.915};
        public static final double[] drivekI = {0, 0, 0, 0};
        public static final double[] drivekD = {0,0,0,0};
        public static final boolean[] driveInversion = {true, true, true, true};
        // public static final boolean[] driveInversion = {false, false, false, false};

        public static final double[] kForwardVolts = {0.129, 0.108, 0.14, 0.125};
        public static final double[] kBackwardVolts = {0.115, 0.169, 0.13, 0.148};

        public static final double[] kForwardVels = {2.910/1.1, 2.970/1.1, 2.890/1.1, 2.930/1.1};
        public static final double[] kBackwardVels = {2.890/1.1, 2.800/1.1, 2.850/1.1, 2.820/1.1};
        public static final double[] kForwardAccels = {0.145, 0.149, 0.192, 0.198};
        public static final double[] kBackwardAccels = {0.192, 0.187, 0.264, 0.176};

        public static final double autoMaxSpeedMps = 0.35 * 4.4;  // Meters / second
        public static final double autoMaxAccelMps2 = mu * g;  // Meters / seconds^2
        public static final double autoMaxVolt = 10.0;   // For Drivetrain voltage constraint in RobotPath.java
        // The maximum acceleration the robot can achieve is equal to the coefficient of static friction times the gravitational acceleration
        // a = mu * 9.8 m/s^2
        public static final double autoCentripetalAccel = mu * g * 2;

        // PID values are listed in the order kP, kI, and kD
        public static final double[] xPIDController = {4, 0.0, 0.0};
        public static final double[] yPIDController = {4, 0.0, 0.0};
        public static final double[] thetaPIDController = {4, 0.0, 0.0};

        public static final SwerveConfig swerveConfig = new SwerveConfig(wheelDiameterMeters, driveGearing, mu, autoCentripetalAccel, kForwardVolts, kForwardVels, kForwardAccels, kBackwardVolts, kBackwardVels, kBackwardAccels, drivekP, drivekI, drivekD, turnkP, turnkI, turnkD, turnkS, turnkV, turnkA, turnZero, driveInversion, reversed);


        public static final int driveFrontLeftPort = 15;
        public static final int driveFrontRightPort = 13;
        public static final int driveBackLeftPort = 5;
        public static final int driveBackRightPort = 4;

        public static final int turnFrontLeftPort = 20;
        public static final int turnFrontRightPort = 14;
        public static final int turnBackLeftPort = 6;
        public static final int turnBackRightPort = 10;

        public static final int canCoderPortFL = 1;
        public static final int canCoderPortFR = 2;
        public static final int canCoderPortBL = 3;
        public static final int canCoderPortBR = 4;


        public static final double kSlowDriveSpeed = 0.25;
        public static final double kSlowDriveRotation = 0.30;
        public static final double kAlignMultiplier = 1D/3D;
        public static final double kAlignForward = 0.6;

    }

    public static final class OI {

        public static final double JOY_THRESH = 0.01;

        public static final class Driver {
            public static final int port = 0;

            public static final int slowDriveButton = 1;
        }
        public static final class Manipulator {
            public static final int port = 1;
        }
    }

}