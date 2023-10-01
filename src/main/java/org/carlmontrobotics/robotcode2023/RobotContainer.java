// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023;

import static org.carlmontrobotics.robotcode2023.Constants.OI.MIN_AXIS_TRIGGER_VALUE;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import org.carlmontrobotics.lib199.Limelight;
import org.carlmontrobotics.lib199.path.PPRobotPath;
import org.carlmontrobotics.robotcode2023.Constants.GoalPos;
import org.carlmontrobotics.robotcode2023.Constants.OI.Driver;
import org.carlmontrobotics.robotcode2023.Constants.OI.Manipulator;
import org.carlmontrobotics.robotcode2023.commands.AlignChargingStation;
import org.carlmontrobotics.robotcode2023.commands.ArmTeleop;
import org.carlmontrobotics.robotcode2023.commands.DriveOverChargeStation;
import org.carlmontrobotics.robotcode2023.commands.RotateToFieldRelativeAngle;
import org.carlmontrobotics.robotcode2023.commands.RunRoller;
import org.carlmontrobotics.robotcode2023.commands.SetArmWristGoalPreset;
import org.carlmontrobotics.robotcode2023.commands.SetArmWristPositionV3;
import org.carlmontrobotics.robotcode2023.commands.TeleopDrive;
import org.carlmontrobotics.robotcode2023.subsystems.Arm;
import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;
import org.carlmontrobotics.robotcode2023.subsystems.Roller;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;



public class RobotContainer {
  
  public static DriverMode driverMode = DriverMode.NORM;

  public final GenericHID driverController = new GenericHID(Driver.port);
  public final GenericHID manipulatorController = new GenericHID(Manipulator.port);

  public final PowerDistribution pd = new PowerDistribution();

  public final Limelight lime = new Limelight();
  public final Drivetrain drivetrain = new Drivetrain();
  public final Arm arm = new Arm();
  public final Roller roller = new Roller();

  public final Command[] autoPaths;
  public final DigitalInput[] autoSelectors;

  public HashMap<String, Command> eventMap;

  public RobotContainer() {

    eventMap = new HashMap<>();

    {
      eventMap.put("Cone High Pos.", new SetArmWristGoalPreset(GoalPos.HIGH, () -> false, () -> false, arm));
      // Command fakeArmCommand = new InstantCommand(() -> System.err.println("==============Store================="), arm);
      // eventMap.put("Stored Pos.", new SequentialCommandGroup(fakeArmCommand, new WaitCommand(2)));
      eventMap.put("Run Cube Intake", new SequentialCommandGroup(new SetArmWristGoalPreset(GoalPos.INTAKE, () -> true, () -> false, arm), new RunRoller(roller, Roller.RollerMode.INTAKE_CUBE)));
      eventMap.put("DriveOverChargeStation", new DriveOverChargeStation(drivetrain));
      eventMap.put("Extend Arm High Cube", new SetArmWristGoalPreset(GoalPos.HIGH, () -> true, () -> false, arm));
      eventMap.put("Extend Arm Mid Cube", new SetArmWristGoalPreset(GoalPos.MID, () -> true, () -> false, arm));
      eventMap.put("Store Arm", new SetArmWristGoalPreset(GoalPos.STORED, () -> true, () -> false, arm));

      eventMap.put("Cube High Pos.", new SequentialCommandGroup(
        new PrintCommand("================================Cube High Pos. Started=================================="),
        new SetArmWristGoalPreset(GoalPos.HIGH, () -> true, () -> false, arm),
        new PrintCommand("================================Cube High Pos. Ended==================================")
        ));
      eventMap.put("Run Cube Outtake", new RunRoller(roller, Roller.RollerMode.OUTTAKE_CUBE));
      eventMap.put("Run Cone Intake", new SequentialCommandGroup(new SetArmWristGoalPreset(GoalPos.INTAKE, () -> false, () -> false, arm), new RunRoller(roller, Roller.RollerMode.INTAKE_CONE)));
      eventMap.put("Run Cone Outtake", new RunRoller(roller, Roller.RollerMode.OUTTAKE_CONE));
      //eventMap.put("Move Arm Back", new SetArmWristPositionV3((-5*Math.PI)/8, Constants.Arm.WRIST_STOW_POS_RAD, arm));
      eventMap.put("Cone Intake Pos.", new SetArmWristGoalPreset(GoalPos.INTAKE, () -> false, () -> false, arm));
      eventMap.put("Cube Intake Pos.", new SequentialCommandGroup(
        new PrintCommand("================================Cube Intake Pos. Started=================================="),
        new SetArmWristGoalPreset(GoalPos.INTAKE, () -> true, () -> false, arm),
        new PrintCommand("================================Cube Intake Pos. Ended==================================")
      ));
      eventMap.put("Field Rotate 90", new RotateToFieldRelativeAngle(new Rotation2d(Math.PI / 2), drivetrain));
      eventMap.put("Rotate 180", new ProxyCommand(new RotateToFieldRelativeAngle(new Rotation2d(Units.degreesToRadians(drivetrain.getHeadingDeg() + 180)), drivetrain)));
      eventMap.put("Stop", stopDt());
      eventMap.put("Auto-Align", new ProxyCommand(() -> new AlignChargingStation(drivetrain)));
      // eventMap.put("PrintAlign", new PrintCommand("Aligning"));
      // eventMap.put("PrintCube", new PrintCommand("Cube"));
      // eventMap.put("PrintStored", new PrintCommand("Stored"));
      // eventMap.put("PrintOne", new PrintCommand("one"));
      // eventMap.put("PrintTwo", new PrintCommand("two"));
      // eventMap.put("PrintEnd", new PrintCommand("end"));
      eventMap.put("Reset Field Orientation", new InstantCommand(drivetrain::resetFieldOrientation));
    }

    autoPaths = new Command[] {
      null,
      new PPRobotPath("Mid Basic", drivetrain, false, eventMap).getPathCommand(true, true),
      new PPRobotPath("Side Basic", drivetrain, false, eventMap).getPathCommand(true, true),
      new PPRobotPath("Place Cone", drivetrain, false, eventMap).getPathCommand(true, true),
      new PPRobotPath(PathPlanner.loadPathGroup("Side Basic 1", drivetrain.getMaxSpeedMps(), 1, false), drivetrain, eventMap)
      .getPathCommand(true, true).andThen(
        new PPRobotPath("Side Basic 2", drivetrain, false, eventMap).getPathCommand(false, true)
      ),
      new SequentialCommandGroup(
        new PPRobotPath("Mid Basic 5-1", drivetrain, false, eventMap).getPathCommand(true, true),
        new ProxyCommand(new RotateToFieldRelativeAngle(new Rotation2d(Units.degreesToRadians(drivetrain.getHeadingDeg() + 180)), drivetrain)),
        new PPRobotPath("Mid Basic 5-2", drivetrain, false, eventMap).getPathCommand(true, true)
      )
    };

    autoSelectors = new DigitalInput[Math.min(autoPaths.length, 26)];
    for(int i = 0; i < autoSelectors.length; i++) autoSelectors[i] = new DigitalInput(i);

    drivetrain.setDefaultCommand(new TeleopDrive(
      drivetrain,
      () -> inputProcessing(getStickValue(driverController, Axis.kLeftY)),
      () -> inputProcessing(getStickValue(driverController, Axis.kLeftX)),
      () -> inputProcessing(getStickValue(driverController, Axis.kRightX)),
      () -> {return driverMode;}
    ));

    configureButtonBindingsDriver();
    configureButtonBindingsManipulator();
    arm.setDefaultCommand(new ArmTeleop(
      arm,
      () -> {return (driverMode.isBaby() ? 0 : inputProcessing(getStickValue(manipulatorController, Axis.kLeftY)));},
      () -> {return (driverMode.isBaby() ? 0 : inputProcessing(getStickValue(manipulatorController, Axis.kRightY)));},
			() -> {return driverMode.isBaby();}
    ));
  }
  private void configureButtonBindingsDriver() {
    new JoystickButton(driverController, Driver.chargeStationAlignButton).onTrue(new AlignChargingStation(drivetrain));
    new JoystickButton(driverController, Driver.resetFieldOrientationButton).onTrue(new InstantCommand(drivetrain::resetFieldOrientation));
    new JoystickButton(driverController, Driver.toggleFieldOrientedButton).onTrue(new InstantCommand(() -> drivetrain.setFieldOriented(!drivetrain.getFieldOriented())));
    new JoystickButton(driverController, Driver.rotateToFieldRelativeAngle0Deg).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(0), drivetrain));
    new JoystickButton(driverController, Driver.rotateToFieldRelativeAngle90Deg).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(-90), drivetrain));
    new JoystickButton(driverController, Driver.rotateToFieldRelativeAngle180Deg).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(180), drivetrain));
    new JoystickButton(driverController, Driver.rotateToFieldRelativeAngle270Deg).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(90), drivetrain));
		
		new JoystickButton(driverController, Driver.slowDriveButton).whileTrue(new InstantCommand(() -> driverMode.isSlow()));
    
  }

  private void configureButtonBindingsManipulator() {
    BooleanSupplier isCube = () -> new JoystickButton(manipulatorController, Manipulator.toggleCubeButton).getAsBoolean();
    BooleanSupplier isFront = () -> new JoystickButton(manipulatorController, Manipulator.toggleFrontButton).getAsBoolean();
    BooleanSupplier isIntake = () -> !isCube.getAsBoolean();

    new JoystickButton(manipulatorController, Manipulator.storePosButton).onTrue(new SetArmWristGoalPreset(GoalPos.STORED, isCube, isFront, arm));
    //new JoystickButton(manipulatorController, Manipulator.lowPosButton).onTrue(new SetArmWristGoalPreset(GoalPos.LOW, isCube, isFront, arm));
    new JoystickButton(manipulatorController, Manipulator.midPosButton).onTrue(new SetArmWristGoalPreset(GoalPos.MID, isCube, isFront, arm));
    new JoystickButton(manipulatorController, Manipulator.highPosButton).onTrue(new SetArmWristGoalPreset(GoalPos.HIGH, isCube, isFront, arm));
    new POVButton(manipulatorController, Manipulator.shelfPickupPOV).onTrue(new SetArmWristGoalPreset(GoalPos.SHELF, isCube, isFront, arm));
    new POVButton(manipulatorController, Manipulator.intakeConePOV).onTrue(new SetArmWristGoalPreset(GoalPos.INTAKE, () -> false, isFront, arm));
    new POVButton(manipulatorController, Manipulator.substationPickupPOV).onTrue(new SetArmWristGoalPreset(GoalPos.STORED, isCube, isFront, arm));
		
    new POVButton(manipulatorController, Manipulator.intakeCubePOV).onTrue(new SetArmWristGoalPreset(GoalPos.INTAKE, () -> true, isFront, arm));
    new POVButton(manipulatorController, Manipulator.intakeConePOV).onTrue(new SetArmWristGoalPreset(GoalPos.INTAKE, () -> false, isFront, arm).andThen(new RunRoller(roller, Roller.RollerMode.INTAKE_CONE)));
    new POVButton(manipulatorController, Manipulator.substationPickupPOV).onTrue(new SetArmWristGoalPreset(GoalPos.SUBSTATION, isCube, isFront, arm).andThen(new RunRoller(roller, Roller.RollerMode.INTAKE_CONE)));
    new POVButton(manipulatorController, Manipulator.intakeCubePOV).onTrue(new SetArmWristGoalPreset(GoalPos.INTAKE, () -> true, isFront, arm).andThen(new RunRoller(roller, Roller.RollerMode.INTAKE_CUBE)));
    new JoystickButton(manipulatorController, Manipulator.stopRollerButton).onTrue(new RunRoller(roller, Roller.RollerMode.STOP));
    
    // axisTrigger(manipulatorController, Manipulator.rollerIntakeConeButton)
    //   .onTrue(new RunRoller(roller, RollerMode.INTAKE_CONE, Constants.Roller.conePickupColor));
    axisTrigger(manipulatorController, Manipulator.rollerIntakeCubeButton)
      .onTrue(new ConditionalCommand(
        new RunRoller(roller, Roller.RollerMode.INTAKE_CUBE), 
        new RunRoller(roller, Roller.RollerMode.OUTTAKE_CUBE), 
        isIntake
      )); 
    axisTrigger(manipulatorController, Manipulator.rollerIntakeConeButton)
      .onTrue(new ConditionalCommand(
        new RunRoller(roller, Roller.RollerMode.INTAKE_CONE), 
        new RunRoller(roller, Roller.RollerMode.OUTTAKE_CONE), 
        isIntake
      ));

  }

  public Command stopDt() {
    return (new InstantCommand(drivetrain::stop).repeatedly()).until(drivetrain::isStopped);
  }

  public Command getAutonomousCommand() {
    Command autoPath = null;
    //PPRobotPath autoPath = new PPRobotPath("Basic 7", drivetrain, false, eventMap);
    // Command[] autoPath2 = {
    //   new PPRobotPath("Mid Basic 3", drivetrain, false, eventMap).getPathCommand(true, true), 
    //   new PPRobotPath("Mid Basic 4", drivetrain, false, eventMap).getPathCommand(false, true)
    // };
    // Command[] commands = {
    //   stopDt(),
    //   new WaitCommand(0)
    // };
    // SequentialCommandGroup autoCommand = new SequentialCommandGroup();
    // for (int i = 0; i < autoPath2.length; i++) {
    //   autoCommand.addCommands(autoPath2[i]);
    //   //autoCommand.addCommands(commands[i]);
    // }

    for(int i = 0; i < autoSelectors.length; i++) {
      if(!autoSelectors[i].get()) {
        System.out.println("Using Path: " + i);
        autoPath = autoPaths[i];
        break;
      }
    }

    //return autoPath == null ? new PrintCommand("No Autonomous Routine selected") : autoCommand;
     return autoPath == null ? new PrintCommand("null :(") : autoPath;
  }


  public void onEnable() {
    lime.getNTEntry("pipeline").setDouble(DriverStation.getAlliance() == Alliance.Red ? 1 : 0);
  }

  private double getStickValue(GenericHID stick, Axis axis) {

    return stick.getRawAxis(axis.value) * (axis == Axis.kLeftY || axis == Axis.kRightY ? -1 : 1);
  }

  /**
   * Processes an input from the joystick into a value between -1 and 1
   * 
   * @param value The value to be processed.
   * @return The processed value.
   */
  private double inputProcessing(double value) {
    double processedInput;
    // processedInput =
    // (((1-Math.cos(value*Math.PI))/2)*((1-Math.cos(value*Math.PI))/2))*(value/Math.abs(value));
    processedInput = Math.copySign(((1 - Math.cos(value * Math.PI)) / 2) * ((1 - Math.cos(value * Math.PI)) / 2),
        value);
    return processedInput;
  }

  /**
   * Returns a new instance of Trigger based on the given Joystick and Axis objects.
   * The Trigger is triggered when the absolute value of the stick value on the specified axis
   * exceeds a minimum threshold value.
   * 
   * @param stick The Joystick object to retrieve stick value from.
   * @param axis The Axis object to retrieve value from the Joystick.
   * @return A new instance of Trigger based on the given Joystick and Axis objects.
   * @throws NullPointerException if either stick or axis is null.
   */
  private Trigger axisTrigger(GenericHID stick, Axis axis) {
    return new Trigger(() -> Math.abs(getStickValue(stick, axis)) > MIN_AXIS_TRIGGER_VALUE);
  }
  
	public static enum DriverMode {
				NORM,SLOW,BABY;
				public boolean isNorm(){ return this==DriverMode.NORM; }
				public boolean isSlow(){ return this==DriverMode.SLOW; }
				public boolean isBaby(){ return this==DriverMode.BABY; }
		}
}
