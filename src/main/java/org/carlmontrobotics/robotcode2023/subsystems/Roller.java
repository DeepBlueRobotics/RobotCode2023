// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.subsystems;

import java.awt.Color;

import org.carlmontrobotics.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import static org.carlmontrobotics.robotcode2023.Constants.Roller.*;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Roller extends SubsystemBase {

    private final CANSparkMax motor = MotorControllerFactory.createSparkMax(rollerPort, MotorConfig.NEO_550);
    private TimeOfFlight distSensor = new TimeOfFlight(10);
    private final AddressableLED led = new AddressableLED(ledPort);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(ledLength);
    private boolean hadGamePiece = false;

    private Command resetColorCommand = new SequentialCommandGroup(
            new WaitCommand(ledDefaultColorRestoreTime),
            new InstantCommand(() -> setLedColor(defaultColor))) {
        public boolean runsWhenDisabled() {
            return true;
        };
    };

    public Roller() {
        led.setLength(ledBuffer.getLength());
        setLedColor(defaultColor);
        motor.setSmartCurrentLimit(ROLLER_MAX_CURRENT_AMPS);
        led.start();
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled())
            setSpeed(0);
            
        SmartDashboard.putBoolean("Has Game Piece", hasGamePiece());
        SmartDashboard.putNumber("Roller Game Piece Distance", getGamePieceDistanceIn());

        // LED Update
        {
            boolean hasGamePiece = hasGamePiece();
            if (hasGamePiece && !hadGamePiece) {
                setLedColor(pickupSuccessColor);
                resetColorCommand.schedule();
            }
            hadGamePiece = hasGamePiece;
        }
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public void setLedColor(Color color) {
        for (int i = 0; i < ledBuffer.getLength(); i++)
            ledBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
        led.setData(ledBuffer);
    }

    public boolean hasGamePiece() {
        //return false;
        return getGamePieceDistanceIn() < gamePieceDetectDistanceIn;
    }

    public double getGamePieceDistanceIn() {
        return Units.metersToInches((distSensor.getRange() - distSensorDepthMM /*
                                                                                * The sensor measures from the back of
                                                                                * the sensor
                                                                                */) / 1000 /* Convert mm to m */);
    }

    public void setRollerMode(RollerMode mode) {
        setSpeed(mode.speed);
        setLedColor(mode.ledColor);
    }

    public void putRollerConstsOnSmartDashboard() {
        SmartDashboard.putNumber("Intake Cone Speed", RollerMode.INTAKE_CONE.speed);
        SmartDashboard.putNumber("Outtake Cone Speed", RollerMode.OUTTAKE_CONE.speed);
        SmartDashboard.putNumber("Intake Cube Speed", RollerMode.INTAKE_CUBE.speed);
        SmartDashboard.putNumber("Outtake Cube Speed", RollerMode.OUTTAKE_CUBE.speed);
        SmartDashboard.putNumber("Intake Cone Time", RollerMode.INTAKE_CONE.time);
        SmartDashboard.putNumber("Outtake Cone Time", RollerMode.OUTTAKE_CONE.time);
        SmartDashboard.putNumber("Intake Cube Time", RollerMode.INTAKE_CUBE.time);
        SmartDashboard.putNumber("Outtake Cube Time", RollerMode.OUTTAKE_CUBE.time);
    }

    public void getRollerConstsOnSmartDashboard() {
        RollerMode.INTAKE_CONE.speed = SmartDashboard.getNumber("Intake Cone Speed", RollerMode.INTAKE_CONE.speed);
        RollerMode.OUTTAKE_CONE.speed = SmartDashboard.getNumber("Outtake Cone Speed", RollerMode.OUTTAKE_CONE.speed);
        RollerMode.INTAKE_CUBE.speed = SmartDashboard.getNumber("Intake Cube Speed", RollerMode.INTAKE_CUBE.speed);
        RollerMode.OUTTAKE_CUBE.speed = SmartDashboard.getNumber("Outtake Cube Speed", RollerMode.OUTTAKE_CUBE.speed);
        RollerMode.INTAKE_CONE.time = SmartDashboard.getNumber("Intake Cone Time", RollerMode.INTAKE_CONE.time);
        RollerMode.OUTTAKE_CONE.time = SmartDashboard.getNumber("Outtake Cone Time", RollerMode.OUTTAKE_CONE.time);
        RollerMode.INTAKE_CUBE.time = SmartDashboard.getNumber("Intake Cube Time", RollerMode.INTAKE_CUBE.time);
        RollerMode.OUTTAKE_CUBE.time = SmartDashboard.getNumber("Outtake Cube Time", RollerMode.OUTTAKE_CUBE.time);
    }

    public double getPosition() {
        return motor.getEncoder().getPosition();
    }
		
    public static class RollerMode {
        public static RollerMode INTAKE_CONE = new RollerMode(-0.5, .5, GameObject.CONE, conePickupColor);
        public static RollerMode INTAKE_CUBE = new RollerMode(0.4, .25, GameObject.CUBE, cubePickupColor);
        // The obj indicates which game object the roller is trying to intake
        // if obj == NONE, that means it is trying to outtake rather than intake
        public static RollerMode OUTTAKE_CONE = new RollerMode(0.5, .5, GameObject.NONE, defaultColor);
        public static RollerMode OUTTAKE_CUBE = new RollerMode(-0.5, .5, GameObject.NONE, defaultColor);
        public static RollerMode STOP = new RollerMode(0, .1, GameObject.NONE, defaultColor);
        public double speed;
        public double time;
        public GameObject obj;
        public Color ledColor;

        /**
         * @param speed  A number between -1 and 1
         * @param time   Amount of time in seconds to keep the motor running after
         *               distance sensor has detected an object
         * @param intake Whether the roller is outtaking or intaking
         */
        public RollerMode(double speed, double time, GameObject obj, Color ledColor) {
                this.speed = speed;
                this.time = time;
                this.obj = obj;
                this.ledColor = ledColor;
        }
    }
}
