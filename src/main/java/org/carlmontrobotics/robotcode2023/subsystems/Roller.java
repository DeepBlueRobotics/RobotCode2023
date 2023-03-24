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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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
    private GameObject hasGamePiece = GameObject.NONE;
    private GameObject nextGamePiece = GameObject.NONE;

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
        // SmartDashboard.putData(this);
        led.start();
    }

    @Override
    public void periodic() {

        SmartDashboard.putBoolean("Has Game Piece", getGamePiece() != GameObject.NONE);
        SmartDashboard.putNumber("Roller Game Piece Distance", getGamePieceDistanceIn());

        // LED Update
        {
            GameObject gamePiece = getGamePiece();
            if (gamePiece != GameObject.NONE && hasGamePiece == GameObject.NONE) {
                setLedColor(pickupSuccessColor);
                resetColorCommand.schedule();
            }
            hasGamePiece = gamePiece;
        }
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public void setRollerMode(RollerMode mode) {
        setSpeed(mode.speed);
        nextGamePiece = mode.obj;
        setLedColor(mode.ledColor);
    }

    public void setLedColor(Color color) {
        for (int i = 0; i < ledBuffer.getLength(); i++)
            ledBuffer.setRGB(i, color.getRed(), color.getGreen(), color.getBlue());
        led.setData(ledBuffer);
    }

    public GameObject getGamePiece() {
        // return false;
        if (getGamePieceDistanceIn() > gamePieceDetectDistanceIn)
            return GameObject.NONE;
        return (nextGamePiece != GameObject.NONE) ? nextGamePiece : hasGamePiece;
    }

    public boolean hasGamePiece() {
        return getGamePiece() != GameObject.NONE;
    }

    public double getGamePieceDistanceIn() {
        return Units.metersToInches((distSensor.getRange() - distSensorDepthMM /*
                                                                                * The sensor measures from the back of
                                                                                * the sensor
                                                                                */) / 1000 /* Convert mm to m */);
    }

    // The robot-relative translation to correct the off-centering of the game object in the roller
    // Measured in meters
    public Translation2d correctPosition() {
        double distanceToMove = 0;
        double dist = getGamePieceDistanceIn();
        double offset = 0;
        if (hasGamePiece == GameObject.CUBE) offset = Units.inchesToMeters(CUBE_RADIUS_IN) / 2;
        else if (hasGamePiece == GameObject.CONE) offset = Units.inchesToMeters(CONE_RADIUS_IN) / 2;

        Translation2d translation = new Translation2d(0, 0);
        if (dist < ROLLER_WIDTH / 2) {
            // need to shift robot right
            distanceToMove = ROLLER_WIDTH / 2 - dist - offset;
            translation = new Translation2d(distanceToMove, new Rotation2d(-Math.PI / 2));

        } else if (dist > ROLLER_WIDTH / 2) {
            // need to shift robot left
            distanceToMove = dist + offset - ROLLER_WIDTH / 2;
            translation = new Translation2d(distanceToMove, new Rotation2d(Math.PI / 2));
        }
        return translation;
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
}
