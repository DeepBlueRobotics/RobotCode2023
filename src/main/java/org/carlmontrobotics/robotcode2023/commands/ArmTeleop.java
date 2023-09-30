// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.commands;

import static org.carlmontrobotics.robotcode2023.Constants.Arm.*;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import org.carlmontrobotics.robotcode2023.Constants;
import org.carlmontrobotics.robotcode2023.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmTeleop extends CommandBase {
  /** Creates a new ArmPeriodic. */
  private Arm armSubsystem;
  private DoubleSupplier arm;
  private DoubleSupplier wrist;
	private BooleanSupplier baby;//is robot in baby (safe) mode?
  private double lastTime = 0;

  public ArmTeleop(Arm armSubsystem, DoubleSupplier arm, DoubleSupplier wrist, BooleanSupplier baby) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.armSubsystem = armSubsystem);
    this.arm = arm;
    this.wrist = wrist;
		this.baby = baby;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.setArmTarget(armSubsystem.getArmPos(), 0);
    armSubsystem.setWristTarget(armSubsystem.getWristPos(), 0);
    lastTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] speeds = getRequestedSpeeds();
    double currTime = Timer.getFPGATimestamp();
    double deltaT = currTime - lastTime;

    double goalArmRad = armSubsystem.getCurrentArmGoal().position + speeds[ARM] * deltaT;
    double goalWristRad = armSubsystem.getCurrentWristGoal().position + speeds[WRIST] * deltaT;

    // Clamp the goal to the limits of the arm and wrist
    // setArm/WristTarget will also do a modulus, so clamp first to avoid wrapping
    goalArmRad = MathUtil.clamp(goalArmRad, ARM_LOWER_LIMIT_RAD, ARM_UPPER_LIMIT_RAD);
    goalWristRad = MathUtil.clamp(goalWristRad, WRIST_LOWER_LIMIT_RAD, WRIST_UPPER_LIMIT_RAD);

    // Clamp the goal to within a certain range of the current position to prevent "lag"
    goalArmRad = MathUtil.clamp(goalArmRad, armSubsystem.getArmPos() - ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD, armSubsystem.getArmPos() + ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD);
    goalWristRad = MathUtil.clamp(goalWristRad, armSubsystem.getWristPos() - ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD, armSubsystem.getWristPos() + ARM_TELEOP_MAX_GOAL_DIFF_FROM_CURRENT_RAD);

    if((speeds[0] != 0 || speeds[1] != 0) && !DriverStation.isAutonomous()) {
      armSubsystem.setArmTarget(goalArmRad, armSubsystem.getCurrentArmGoal().velocity);
      armSubsystem.setWristTarget(goalWristRad, armSubsystem.getCurrentWristGoal().velocity);
    }

    lastTime = currTime;
  }

  // Copy and pasted from drivetrain, handles input from joysticks
  public double[] getRequestedSpeeds() {
		// Sets all values less than or equal to a very small value (determined by the
    // idle joystick state) to zero.
		
    double rawArmVel, rawWristVel;
		// prep maximum velocity variable
		double[] maxFF = baby.getAsBoolean() ? MAX_FF_VEL_BABY : MAX_FF_VEL_MANUAL;
    
    if (Math.abs(arm.getAsDouble()) <= Constants.OI.JOY_THRESH)
      rawArmVel = 0.0;
    else
      rawArmVel = maxFF[ARM] * arm.getAsDouble();

    if (Math.abs(wrist.getAsDouble()) <= Constants.OI.JOY_THRESH)
      rawWristVel = 0.0;
    else
      rawWristVel = maxFF[WRIST] * wrist.getAsDouble();

    return new double[] {rawArmVel, rawWristVel};
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
