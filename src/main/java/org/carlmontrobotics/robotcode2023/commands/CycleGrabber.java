// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.commands;

import org.carlmontrobotics.robotcode2023.Constants;
import org.carlmontrobotics.robotcode2023.subsystems.Grabber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CycleGrabber extends CommandBase {

  private boolean opening = false;

  public CycleGrabber(Grabber grabber) {
    addRequirements(grabber);
  }

  @Override
  public void initialize() {
    opening = SmartDashboard.getNumber("Motor Position", 0) < Constants.grabber_open_position;
  }

  @Override
  public void execute() {
    if (opening) {
      SmartDashboard.putNumber("Motor Voltage", SmartDashboard.getNumber("Open Speed", 0.05));
    } else {
      SmartDashboard.putNumber("Motor Voltage", SmartDashboard.getNumber("Close Speed", -0.05));
    }
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("Motor Voltage", 0);
    SmartDashboard.putBoolean("open and close", false);
  }

  @Override
  public boolean isFinished() {
    return (opening && SmartDashboard.getNumber("Motor Position", 0) >= Constants.grabber_open_position) ||
    (!opening && SmartDashboard.getNumber("Motor Position", 0) <= Constants.grabber_closed_position);
  }
}
