/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.carlmontrobotics.robotcode2023.commands;

import static org.carlmontrobotics.robotcode2023.Constants.Drivetrain.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.carlmontrobotics.robotcode2023.Constants;
import org.carlmontrobotics.robotcode2023.Robot;
import org.carlmontrobotics.robotcode2023.subsystems.Arm;
import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrive extends CommandBase {

  private static final double robotPeriod = Robot.robot.getPeriod();
  private final Drivetrain drivetrain;
  private final DoubleSupplier fwd;
  private final DoubleSupplier str;
  private final DoubleSupplier rcw;
  private final BooleanSupplier slow;
  private final Arm arm;
  private double currentForwardVel = 0;
  private double currentStrafeVel = 0;

  /**
   * Creates a new TeleopDrive.
   */
  public TeleopDrive(Drivetrain drivetrain, DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rcw, BooleanSupplier slow, Arm arm) {
    addRequirements(this.drivetrain = drivetrain);
    this.fwd = fwd;
    this.str = str;
    this.rcw = rcw;
    this.slow = slow;
    this.arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] speeds = getRequestedSpeeds();
    drivetrain.drive(speeds[0], speeds[1], speeds[2]);
  }

  public double[] getRequestedSpeeds() {
    // Sets all values less than or equal to a very small value (determined by the idle joystick state) to zero.
    // Used to make sure that the robot does not try to change its angle unless it is moving,
    double forward = fwd.getAsDouble();
    double strafe = str.getAsDouble();
    double rotateClockwise = rcw.getAsDouble();
    if (Math.abs(forward) <= Constants.OI.JOY_THRESH) forward = 0.0;
    else forward *= maxForward;
    if (Math.abs(strafe) <= Constants.OI.JOY_THRESH) strafe = 0.0;
    else strafe *= maxStrafe;
    if (Math.abs(rotateClockwise) <= Constants.OI.JOY_THRESH) rotateClockwise = 0.0;
    else rotateClockwise *= maxRCW;

    double driveMultiplier = slow.getAsBoolean() ? kSlowDriveSpeed : kNormalDriveSpeed;
    double rotationMultiplier = slow.getAsBoolean() ? kSlowDriveRotation : kNormalDriveRotation;

    forward *= driveMultiplier;
    strafe *= driveMultiplier;
    rotateClockwise *= rotationMultiplier;

    // Limit acceleration of the robot
    double accelerationX = (forward - currentForwardVel) / robotPeriod;
    double accelerationY = (strafe - currentStrafeVel) / robotPeriod;

    double maxXAccel = (Math.signum(accelerationX) == Math.signum(getCoM().getX()) ? getForwardNearAccelLimit() : getForwardFarAccelLimit());
    if(Math.abs(accelerationX) > maxXAccel) accelerationX = Math.copySign(maxXAccel, accelerationX);
    double maxYAccel = getStrafeAccelLimit();
    if(Math.abs(accelerationY) > maxYAccel) accelerationY = Math.copySign(maxYAccel, accelerationY);

    currentForwardVel += accelerationX * robotPeriod;
    currentStrafeVel += accelerationY * robotPeriod;

    // double translationalAcceleration = Math.hypot(accelerationX, accelerationY);
    // if(translationalAcceleration > autoMaxAccelMps2) {
    //   Translation2d limitedAccelerationVector = new Translation2d(autoMaxAccelMps2, Rotation2d.fromRadians(Math.atan2(accelerationY, accelerationX)));
    //   Translation2d limitedVelocityVector = limitedAccelerationVector.times(robotPeriod);
    //   currentForwardVel += limitedVelocityVector.getX();
    //   currentStrafeVel += limitedVelocityVector.getY();
    // } else {
    //   currentForwardVel = forward;
    //   currentStrafeVel = strafe;
    // }

    // ATM, there is no rotational acceleration limit

    // If the above math works, no velocity should be greater than the max velocity, so we don't need to limit it.

    // Limit rotation speed
    double maxRotation = getMaxRotationSpeed();
    if(Math.abs(rotateClockwise) > maxRotation) rotateClockwise = Math.copySign(maxRotation, rotateClockwise);

    return new double[] {currentForwardVel, currentStrafeVel, -rotateClockwise};
  }

  // CoM with origin at robot center on floor
  public Translation2d getCoM() {
    Translation2d comArm = arm.getCoM().plus(Constants.Arm.ARM_JOINT_POS);

    return comArm.times(Constants.Arm.ARM_MASS_KG + Constants.Arm.ROLLER_MASS_KG).plus(COM_ROBOT.times(ROBOT_MASS - Constants.Arm.ARM_MASS_KG - Constants.Arm.ROLLER_MASS_KG));
  }

  // Gets the acceleration limit in the direction opposite the CoM
  public double getForwardFarAccelLimit() {
    Translation2d com = getCoM();

    return Math.abs(wheelBase / 2 - com.getX()) * Constants.g / com.getY();
  }

  // Gets the acceleration limit in the direction opposite the CoM
  public double getForwardNearAccelLimit() {
    Translation2d com = getCoM();

    return Math.abs(wheelBase / 2 + com.getX()) * Constants.g / com.getY();
  }

  public double getStrafeAccelLimit() {
    Translation2d com = getCoM();

    return trackWidth * Constants.g / com.getY();
  }

  public double getMaxRotationSpeed() {
    Translation2d com = getCoM();

    return Math.sqrt(wheelBase / com.getY());
  }
  public boolean hasDriverInput() {
    return Math.abs(fwd.getAsDouble()) > Constants.OI.JOY_THRESH || Math.abs(str.getAsDouble()) > Constants.OI.JOY_THRESH || Math.abs(rcw.getAsDouble()) > Constants.OI.JOY_THRESH;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
