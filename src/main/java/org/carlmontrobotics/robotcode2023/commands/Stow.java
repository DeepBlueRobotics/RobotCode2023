// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.robotcode2023.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.carlmontrobotics.robotcode2023.subsystems.Arm;

public class Stow extends CommandBase {
  
  

  /** Creates a new Stow. */
  
  public Stow(Arm aram) {
    // Use addRequirements() here to declare subsystem dependencies.
    
   final double wristStowPosB = Units.degreesToRadians(-180);
   final double wristStowPosF = Units.degreesToRadians(180);
  
    if(aram.isInside(true))
    {
      if( aram.getWristPos() < Units.degreesToRadians(-90))
      {
        aram.setWristTarget(wristStowPosB);
        
      }
      else if (aram.getWristPos() > Units.degreesToRadians(-90))
      {
        aram.setWristTarget(wristStowPosF);
      }
    }
    
    
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
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
    return false;
  }
}
