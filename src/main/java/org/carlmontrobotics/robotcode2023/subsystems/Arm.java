package org.carlmontrobotics.robotcode2023.subsystems;

import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax;



public class Arm extends SubsystemBase {
  
  public CANSparkMax motor = MotorControllerFactory.createSparkMax(Constants.Arm.port, TemperatureLimit.NEO);
  public SparkMaxAbsoluteEncoder encoder;

  public double encoderErrorTolerance = .05;

  private double kS = .067766; //volts | base speed
  private double kG = .0075982; //volts | gravity... something
  private double kV = .019762; //volts*secs/rad | extra velocity
  private double kA = .00039212; //volts*secs^2/rad | vacceleration
  /// these are all units ^ , actual arm speed is determined by values in .calculate
  private double kP = 1.1;
  private double kI = 1.1; //will add real values
  private double kD = 1.1;

  private double FFvelocity = .01;
  private double FFaccel = .01;
  private ArmFeedforward armFeed = new ArmFeedforward(kS, kG, kV, kA);
  private PIDController pid = new PIDController(kP, kI, kD);
  
  public ArmPreset goalEnum;
  private double EncoderPos = encoder.getZeroOffset();
    
  public double hiClamp = -Math.PI*.5; //TODO GET NUMBERS
  public double loClamp = -Math.PI*1.4;

  public enum ArmPreset {
    INTAKE(0.31), MID(-1.74), HIGH(-1.83);
    
    public double value; //not static so SmartDashboard can touch [IMPORTANT TO KNOW!]
    private double origin;
    ArmPreset(double value) {
      this.value = value;
      this.origin = value;
    }
    private void reset(){
        this.value = this.origin;//for when we manually set the goalEnum pos, when we cycle this will fix itself
    }
    public ArmPreset next() { this.reset(); return this.values()[(this.ordinal()+1) % this.values().length]; }
    public ArmPreset prev() { this.reset(); return this.values()[(this.ordinal()+2) % this.values().length]; }
    //                           -1 but then +3 so that looping works correctly^
      
    //ordinal : after adding : after mod operator
    //next() 0:1:1, 1:2:2, 2:3:0
    //prev() 0:2:2, 1:3:0, 2:4:1 
  }

  public Arm() {
    encoder.setPositionConversionFactor(1/60);
    encoder.getZeroOffset();
    pid.setTolerance(2.5,10);
    
    SmartDashboard.putNumber("GoalPosition", goalEnum.value);
  }

  @Override
  public void periodic() {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
     double currentPos = encoder.getZeroOffset();
    
    double goalPos = MathUtil.clamp(goalEnum.value, loClamp, hiClamp);
      
    motor.setVoltage(armFeed.calculate(currentPos, 0, 0)
      + pid.calculate(currentPos, goalPos));

    goalPos = SmartDashboard.getNumber("GoalPosition", goalPos);
    SmartDashboard.getNumber("EncoderPos", currentPos);
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("FF: Velocity", () -> FFvelocity, x -> this.FFvelocity = x);
    builder.addDoubleProperty("FF: Accel",    () -> FFaccel,    x -> this.FFaccel = x);
    builder.addDoubleProperty("goalPos",      () -> goalEnum.value,    x -> this.goalEnum.value = x);
    builder.addDoubleProperty("EncoderPos",   () -> EncoderPos, x -> this.EncoderPos = x);
    builder.addDoubleProperty("kP",           () -> kP,         x -> this.kP = x);
    builder.addDoubleProperty("kI",           () -> kI,         x -> this.kI = x);
    builder.addDoubleProperty("kD",           () -> kD,         x -> this.kD = x);
    builder.addDoubleProperty("kV",           () -> kV,         x -> this.kV = x);
    builder.addDoubleProperty("kG",           () -> kG,         x -> this.kG = x);
    builder.addDoubleProperty("kS",           () -> kS,         x -> this.kS = x);
    builder.addDoubleProperty("kA",           () -> kA,         x -> this.kA = x);
}
  
  public void setPreset(ArmPreset preset) {
   // SmartDashboard.putNumber("GoalPosition", preset.value);
  }
    
  //Snaps raw encoder pos to one of our cycle positions
  public ArmPreset snappedArmPos() {
    double encoderPos = encoder.getZeroOffset();

    for(ArmPreset check : ArmPreset.values()) {
      double lowdist = (check.value - check.prev().value) / 2;
      double hidist = (check.next().value - check.value) / 2; // get the halfway points between each position and it's neighbors
      if (check.value - lowdist < encoderPos && encoderPos < check.value + hidist) {
        //seperate high and low instead of ABS because maybe difference isn't constant between each position of arm
        //and yes it still works for lowest and highest value
        return check;
      }
    }
    //help something went REALLY wrong
    return null;
  }

  public ArmPreset closeSnappedArmPos() {//more precise snapping
    double encoderPos = encoder.getZeroOffset();

    for(ArmPreset check : ArmPreset.values()) {
        if (Math.abs(check.value - encoderPos) > encoderErrorTolerance) {//maybe will break if cone/cube values are close, but if they are close then lower error or only use one enum
        return check;
        }
    }
    //nothing close enough
    return null;
  }
}
