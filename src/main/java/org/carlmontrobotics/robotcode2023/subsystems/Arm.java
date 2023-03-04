package org.carlmontrobotics.robotcode2023.subsystems;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

public class Arm extends SubsystemBase {
  
  private CANSparkMax motor = MotorControllerFactory.createSparkMax(17, TemperatureLimit.NEO);
  private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  private double gearRatio = 225;
  private double lLimit = 0, rLimit = 0;
  private boolean setLimit = false;

  public Arm() {
    SmartDashboard.putNumber("Motor Voltage", 0);
    encoder.setPositionConversionFactor(1 / gearRatio * 2 * Math.PI);
    motor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Pos", encoder.getPosition());
    double speed = SmartDashboard.getNumber("Motor Voltage", 0);
    motor.set(speed);
    /* 
    if (setLimit && (encoder.getPosition() <= lLimit || encoder.getPosition() >= rLimit)) {
      motor.set(0);
    } else {
    }
    */
  }
}
