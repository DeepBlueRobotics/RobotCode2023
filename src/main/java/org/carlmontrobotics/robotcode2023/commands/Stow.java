// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.commands;

import org.carlmontrobotics.robotcode2023.Constants;
import org.carlmontrobotics.robotcode2023.subsystems.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Stow extends CommandBase {
  
   private final double wristStow = Constants.Arm.wristStowPos;
    
   private final double wristStowPosB = Units.degreesToRadians(-wristStow);
   private final double wristStowPosF = Units.degreesToRadians(wristStow);
   private final double armStow = Units.degreesToRadians(Constants.Arm.ARM_STOW_ANGLE);

   Arm arm = new Arm();
  

  /** Creates a new Stow. */
  
  public Stow(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.arm = arm);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (arm.isInside()) {
      if ( arm.getWristPos() < Units.degreesToRadians(-90)) {
        arm.setArmTarget(-Math.PI + armStow);
        
      }
      else if (arm.getWristPos() > Units.degreesToRadians(-90)) {
        arm.setArmTarget(-armStow);
      }
    }
    else 
    {
      if( arm.getWristPos() < Units.degreesToRadians(-90))
      {
        arm.setWristTarget(wristStowPosB);
        
      }
      else if (arm.getWristPos() > Units.degreesToRadians(-90))
      {
        arm.setWristTarget(wristStowPosF);
      }
    }
    
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.getWristPos() < wristStow - 5 && arm.getWristPos() > -wristStow + 5;
  }
}
