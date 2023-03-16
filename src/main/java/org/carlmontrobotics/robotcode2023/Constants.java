// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

import java.awt.Color;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController.Button;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Arm {

        //#region Subsystem Constants

        // Feedforward
        // FIXME WRIST NEEDS SYSID DONE
        // Arm, Wrist
        public static final double[] kS = { .067766, .074798 }; // (V)
        public static final double[] kV = { .019762, 1.6743 }; // (V / rad/s)
        public static final double[] kA = { .00039212, 0.032177 }; // (V / rad/s^2)
        public static final double kG_WRIST = .36214; // (V)

        // PID
        // FIXME BOTH WRIST AND ARM NEED PID DONE
        // Arm, Wrist
        public static double[] kP = { 0, 0 }; // (V / rad)
        public static double[] kI = { 0, 0 }; // (V / (rad * s) )
        public static double[] kD = { 0, 0 }; // (V / (rad / s) )

        // Arm, Wrist
        public static double[] posToleranceRad = { .05, .05 }; // rad
        public static double[] velToleranceRadPSec = { 0.5, 0.5 }; // rad/s

        public static double[] goalPosRad = { -Math.PI / 2, 0 }; // rad
        public static double[] offsetRad = { 2.08, 4.02 }; // rad

        // needed to calculate feedforward values dynamically
        public static final double ARM_MASS_KG = Units.lbsToKilograms(6.57);
        public static final double ARM_LENGTH_METERS = Units.inchesToMeters(38.25);

        // Distance from the arm motor to the center of mass of the  arm
        public static final double COM_ARM_LENGTH_METERS = Units.inchesToMeters(13.23);
        public static final double ROLLER_MASS_KG = Units.lbsToKilograms(10.91);

        // distance of center of mass of roller to the WRIST motor
        public static final double COM_ROLLER_LENGTH_METERS = Units.inchesToMeters(9.47);
        public static final double g = 9.81;

        public static final double V_PER_NM = 0;

        public static final double ARM_ANGLE_MIN = -3 * Math.PI / 2;
        public static final double ARM_ANGLE_MAX = Math.PI / 2;
        // how far it is from the min/max until it should stop the arm so that arm doesn't break robot
        public static final double ARM_ANGLE_TOLERANCE = Math.PI / 2;
        public static final double WRIST_ANGLE_MIN = -Math.PI;
        public static final double WRIST_ANGLE_MAX = Math.PI;
        // how far it is from the min/max until it should stop the wrist so that wrist doesn't break robot
        public static final double WRIST_ANGLE_TOLERANCE = Math.PI / 6;

        public static double MAX_FF_VEL = 0.1; // rad / s
        public static double MAX_FF_ACCEL = 0.11; // rad / s
        //#endregion

        //#region Ports

        public static final int armMotorPort = 17;
        public static final int wristMotorPort = 19;

        //#endregion

    }
    public static final class Wrist {

    }
    public static final class GoalPos {

        // copy pasted from Wrist branch - These positions seem incorrect
        /*
        // arm
        GROUND(new double[] {1.71254781475, 2.29021388861}), // 98 deg
        LOW(new double[]    {1.02603845343, 4.10094074984}), //59 deg
        MID(new double[]    {0.83753504023, 1.13715693466}), //48 deg
        HIGH(new double[]   {0.76831247212, 1.46030263853}), // 44 deg
        HOLD(new double[] {0.f,0.f}); // 0 deg

        // wrist
        GROUND(new double[] {0.40851176500, 0.408513963836}),
        LOW(new double[]    {-0.9225673417, -0.18997011808}),
        MID(new double[]    {-1.5006099006, -2.11308267430}),
        HIGH(new double[]   {-1.8861790155, -1.86498478435}),
        HOLD(new double[] {0.f,0.f});
        */

        // TODO: PUT IN CORRECT POSITIONS
        // These positions are for if the intake/outtake takes place on the front (battery) side of the robot
        // if intake/outtake on back, the "negative" pos will be used
        // 0 = CUBE, 1 = CONE
        public static GoalPos[] LOW = {new GoalPos(40, -0.9225673417), new GoalPos(40, -0.9225673417)};
        public static GoalPos[] MID = {new GoalPos(40, -1.5006099006), new GoalPos(40, -1.5006099006)};
        public static GoalPos[] HIGH = {new GoalPos(Units.degreesToRadians(11), -1.8861790155), new GoalPos(40, -1.8861790155)};
        public static GoalPos[] STORED = {new GoalPos(Units.degreesToRadians(-90), 0), new GoalPos(40, -1.8861790155)};
        public static GoalPos[] SHELF = {new GoalPos(Units.degreesToRadians(-90), 0), new GoalPos(0, 0)};
        public static GoalPos[] SUBSTATION = {new GoalPos(Units.degreesToRadians(-90), 0), new GoalPos(0, 0)};
        public static GoalPos[] INTAKE = {new GoalPos(Units.degreesToRadians(-90), 0), new GoalPos(Units.degreesToRadians(-90), 0)};

        public double armPos, wristPos;
    
        public GoalPos(double armPos, double wristPos) {
            this.armPos = armPos;
            this.wristPos = wristPos;
        }
    
    }

    public static final class Roller {
        //#region Subsystem Constants

        public static final int ledLength = 85;
        public static final double ledDefaultColorRestoreTime = 3; // The time in seconds after picking up a game piece to restore the LED color to defaultColor
        public static final Color defaultColor = new Color(0, 0, 200);
        public static final Color pickupSuccessColor = new Color(0, 200, 0);
        public static final Color conePickupColor = new Color(150, 150, 0);
        public static final Color cubePickupColor = new Color(50, 0, 200);

        public static final double distSensorDepthMM = 16;
        public static final double gamePieceDetectDistanceIn = 20;

        //#endregion


        //#region Ports
        public static final int rollerPort = 18;
        public static final int ledPort = 0;

        //#endregion



        //#region Command Constants

        public static final double coneIntakeConeOuttakeSpeed = -.3;
        public static final double coneOuttakeConeIntakeSpeed = .7;

        //#endregion
    }

    public static final class OI {
        public static final double JOY_THRESH = 0.01;
        public static final class Driver {
            public static final int port = 0;

        }

        public static final class Manipulator {
            public static final int port = 1;

            public static final int toggleCubeCone = Button.kLeftBumper.value;
            public static final int toggleFrontBack = Button.kRightBumper.value;
            public static final int store = Button.kA.value;
            public static final int low = Button.kX.value;
            public static final int mid = Button.kY.value;
            public static final int high = Button.kB.value;
            // TODO: Determine real ports for the following buttons. I couldn't find what buttons referred to the "4-pad"
            public static final int shelf = -1;
            public static final int intake = -1;
            public static final int substation = -1;

            public static final int rollerIntakeConeButton = Button.kA.value;
            public static final int rollerOuttakeConeButton = Button.kB.value;

            public static final int rollerIntakeCubeButton = Button.kX.value;
            public static final int rollerOuttakeCubeButton = Button.kY.value;
            public static final int rollerStopButton = Button.kRightBumper.value;
        }
    }
}