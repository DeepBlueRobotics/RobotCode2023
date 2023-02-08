package org.carlmontrobotics.robotcode2023.subsystems;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ArmFeedforward;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.carlmontrobotics.robotcode2023.subsystems.Arm; 
public class Wrist extends SubsystemBase{
    private CANSparkMax motorL = MotorControllerFactory.createSparkMax(Constants.wrist_motorL_port, TemperatureLimit.NEO);
    private CANSparkMax motorR = MotorControllerFactory.createSparkMax(Constants.wrist_motorL_port, TemperatureLimit.NEO);
    public RelativeEncoder motorLEncoder = motorL.getEncoder();
    public RelativeEncoder motorREncoder = motorR.getEncoder();
    public double encoderErrorTolerance = .05;
    public double goalPos;


public Wrist(){
    motorL.follow(Arm.motorL, false);
    motorR.follow(motorL, true);
    SmartDashboard.putNumber("GoalPosition", goalPos);
    SmartDashboard.getNumber("GoalPosition", goalPos);

}
}


