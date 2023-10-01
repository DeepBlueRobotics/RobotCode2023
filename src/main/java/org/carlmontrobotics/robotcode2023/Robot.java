// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

import com.pathplanner.lib.server.PathPlannerServer;

import org.carlmontrobotics.lib199.MotorErrors;
import org.carlmontrobotics.lib199.sim.MockedSparkEncoder;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  public static Robot robot;
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    robot = this;
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    if(!DriverStation.isFMSAttached()) PathPlannerServer.startServer(5811);
    robotContainer = new RobotContainer();

    SmartDashboard.putBoolean("safeMode", false);
  }

  @Override
  public void simulationInit() {
    MockedSparkEncoder.setGearing(Constants.Drivetrain.driveFrontLeftPort, Constants.Drivetrain.driveGearing);
    MockedSparkEncoder.setGearing(Constants.Drivetrain.driveFrontLeftPort, Constants.Drivetrain.driveGearing);
    MockedSparkEncoder.setGearing(Constants.Drivetrain.driveFrontLeftPort, Constants.Drivetrain.driveGearing);
    MockedSparkEncoder.setGearing(Constants.Drivetrain.driveFrontLeftPort, Constants.Drivetrain.driveGearing);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    MotorErrors.printSparkMaxErrorMessages();

    //set mode to baby if baby was true
		//else but don't set mode to norm if slow was true
		if (SmartDashboard.getBoolean("safeMode", false)) {
			RobotContainer.driverMode = RobotContainer.DriverMode.BABY;
		} else if (RobotContainer.driverMode.isSlow()) {
			RobotContainer.driverMode = RobotContainer.DriverMode.NORM;
		}
  }


  @Override
  public void disabledInit() {
    new Thread(() -> {
      try {
        Thread.sleep(5000);
      } catch (InterruptedException e) {
        e.printStackTrace();
        return;
      }

      robotContainer.drivetrain.coast();
    }).start();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer.drivetrain.brake();
    robotContainer.getAutonomousCommand().schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer.drivetrain.brake();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer.drivetrain.brake();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
