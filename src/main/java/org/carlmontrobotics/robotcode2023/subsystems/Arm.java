package org.carlmontrobotics.robotcode2023.subsystems;

import static org.carlmontrobotics.robotcode2023.Constants.Arm.*;

import org.carlmontrobotics.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.robotcode2023.Constants.GoalPos;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Arm angle is measured from horizontal on the intake side of the robot and bounded between -3π/2 and π/2
// Wrist angle is measured relative to the arm with 0 being parallel to the arm and bounded between -π and π (Center of Mass of Roller)
public class Arm extends SubsystemBase {

    private final CANSparkMax armMotor = MotorControllerFactory.createSparkMax(armMotorPort, MotorConfig.NEO);
    private final CANSparkMax wristMotor = MotorControllerFactory.createSparkMax(wristMotorPort, MotorConfig.NEO);
    private final SparkMaxAbsoluteEncoder armEncoder = armMotor
            .getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    private final SparkMaxAbsoluteEncoder wristEncoder = wristMotor
            .getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    private final SimpleMotorFeedforward armFeed = new SimpleMotorFeedforward(kS[ARM], kV[ARM], kA[ARM]);
    private ArmFeedforward wristFeed = new ArmFeedforward(kS[WRIST], kG[WRIST], kV[WRIST], kA[WRIST]);

    private PIDController armPID = new PIDController(kP[ARM], kI[ARM], kD[ARM]);
    private PIDController wristPID = new PIDController(kP[WRIST], kI[WRIST], kD[WRIST]);

    private int object = CUBE;
    private boolean isFront = true;

    public Arm() {
        armMotor.setInverted(inverted[ARM]);
        wristMotor.setInverted(inverted[WRIST]);

        armEncoder.setPositionConversionFactor(rotationToRad);
        wristEncoder.setPositionConversionFactor(rotationToRad);

        wristEncoder.setZeroOffset(offsetRad[ARM]);
        armEncoder.setZeroOffset(offsetRad[WRIST]);

        armPID.setTolerance(posToleranceRad[ARM], velToleranceRadPSec[ARM]);
        wristPID.setTolerance(posToleranceRad[WRIST], velToleranceRadPSec[WRIST]);

        putArmControlOnSmartDashboard();
        putCubePositionsOnSmartDashboard();
        putPIDOnSmartDashboard();
        putConePositionsOnSmartDashboard();
        SmartDashboard.putBoolean("Toggle", false);
    }

    @Override
    public void periodic() {
        // TODO: REMOVE THIS WHEN PID CONSTANTS ARE DONE
        armPID.setP(kP[ARM]);
        armPID.setI(kI[ARM]);
        armPID.setD(kD[ARM]);
        wristPID.setP(kP[WRIST]);
        wristPID.setI(kI[WRIST]);
        wristPID.setD(kD[WRIST]);
        if (SmartDashboard.getBoolean("Toggle", false)) {
            getArmControlOnSmartDashboard();
            getPIDOnSmartDashboard();
            getConePositionsOnSmartDashboard();
            getCubePositionsOnSmartDashboard();
        }
    }

    public void driveArm(double vel, double accel) {
        double armFeedVolts = getKg() * getCoM().getAngle().getCos() + armFeed.calculate(vel, accel);
        double armPIDVolts = armPID.calculate(armEncoder.getPosition(), goalPosRad[ARM]);
        if (getArmPos() > ARM_UPPER_LIMIT_RAD || getArmPos() < ARM_LOWER_LIMIT_RAD) {
            armFeedVolts = getKg() * getCoM().getAngle().getCos() + armFeed.calculate(0, 0);
        }
        // TODO: REMOVE WHEN DONE WITH TESTING (ANY CODE REVIEWERS, PLEASE REJECT MERGES
        // TO MASTER IF THIS IS STILL HERE)
        SmartDashboard.putNumber("ArmFeedVolts", armFeedVolts);
        SmartDashboard.putNumber("ArmPIDVolts", armPIDVolts);
        double volts = armFeedVolts + armPIDVolts;
        SmartDashboard.putNumber("ArmTotalVolts", volts);
        armMotor.setVoltage(volts);
    }

    public void driveWrist(double vel, double accel) {
        double wristFeedVolts = wristFeed.calculate(getWristPosRelativeToGround(), vel, accel);
        double wristPIDVolts = wristPID.calculate(getWristPos(), goalPosRad[WRIST]);
        if (getWristPos() > WRIST_UPPER_LIMIT_RAD || getWristPos() < WRIST_LOWER_LIMIT_RAD) {
            wristFeedVolts = wristFeed.calculate(getWristPosRelativeToGround(), 0, 0);
        }
        // TODO: REMOVE WHEN DONE WITH TESTING (ANY CODE REVIEWERS, PLEASE REJECT MERGES
        // TO MASTER IF THIS IS STILL HERE)
        SmartDashboard.putNumber("WristFeedVolts", wristFeedVolts);
        SmartDashboard.putNumber("WristPIDVolts", wristPIDVolts);
        double volts = wristFeedVolts + wristPIDVolts;
        SmartDashboard.putNumber("WristTotalVolts", volts);
        wristMotor.setVoltage(volts);
    }

    public void driveArm(TrapezoidProfile.State profile) {
        double armFeedVolts = getKg() * getCoM().getAngle().getCos() + armFeed.calculate(profile.velocity, 0);
        double armPIDVolts = armPID.calculate(armEncoder.getPosition(), profile.position);
        if (getArmPos() > ARM_UPPER_LIMIT_RAD || getArmPos() < ARM_LOWER_LIMIT_RAD) {
            armFeedVolts = getKg() * getCoM().getAngle().getCos() + armFeed.calculate(0, 0);
        }
        // TODO: REMOVE WHEN DONE WITH TESTING (ANY CODE REVIEWERS, PLEASE REJECT MERGES
        // TO MASTER IF THIS IS STILL HERE)
        SmartDashboard.putNumber("ArmFeedVolts", armFeedVolts);
        SmartDashboard.putNumber("ArmPIDVolts", armPIDVolts);
        double volts = armFeedVolts + armPIDVolts;
        SmartDashboard.putNumber("ArmTotalVolts", volts);
        armMotor.setVoltage(volts);
    }

    public void driveWrist(TrapezoidProfile.State profile) {
        double wristFeedVolts = getKg() * getCoM().getAngle().getCos() + wristFeed.calculate(profile.velocity, 0);
        double wristPIDVolts = wristPID.calculate(wristEncoder.getPosition(), profile.position);
        if (getWristPos() > WRIST_UPPER_LIMIT_RAD || getWristPos() < WRIST_LOWER_LIMIT_RAD) {
            wristFeedVolts = getKg() * getCoM().getAngle().getCos() + wristFeed.calculate(0, 0);
        }
        // TODO: REMOVE WHEN DONE WITH TESTING (ANY CODE REVIEWERS, PLEASE REJECT MERGES
        // TO MASTER IF THIS IS STILL HERE)
        SmartDashboard.putNumber("WristFeedVolts", wristFeedVolts);
        SmartDashboard.putNumber("WristPIDVolts", wristPIDVolts);
        double volts = wristFeedVolts + wristPIDVolts;
        SmartDashboard.putNumber("WristTotalVolts", volts);
        wristMotor.setVoltage(volts);
    }

    public double getArmClampedGoal(double goal) {
        return MathUtil.clamp(MathUtil.inputModulus(goal, ARM_DISCONTINUITY_RAD, ARM_DISCONTINUITY_RAD + 2 * Math.PI), ARM_LOWER_LIMIT_RAD, ARM_UPPER_LIMIT_RAD);
    }
    
    public double getWristClampedGoal(double goal) {
        return MathUtil.clamp(MathUtil.inputModulus(goal, WRIST_DISCONTINUITY_RAD, WRIST_DISCONTINUITY_RAD + 2 * Math.PI), WRIST_LOWER_LIMIT_RAD, WRIST_UPPER_LIMIT_RAD);
    }

    // distance from center of mass relative to joint holding arm
    public Translation2d getCoM() {
        Translation2d comOfArm = new Translation2d(COM_ARM_LENGTH_METERS, Rotation2d.fromRadians(getArmPos()))
                .times(ARM_MASS_KG);
        Translation2d comOfRoller = new Translation2d(ARM_LENGTH_METERS, Rotation2d.fromRadians(getArmPos()))
                .plus(new Translation2d(COM_ROLLER_LENGTH_METERS,
                        Rotation2d.fromRadians(getWristPosRelativeToGround())))
                .times(ROLLER_MASS_KG);
        return comOfArm.plus(comOfRoller).div(ARM_MASS_KG + ROLLER_MASS_KG);
    }

    public double maxHoldingTorqueNM() {
        return (ARM_MASS_KG + ROLLER_MASS_KG) * g * getCoM().getNorm();
    }

    public double getKg() {
        return V_PER_NM * maxHoldingTorqueNM();
    }

    public double getArmPos() {
        return MathUtil.inputModulus(armEncoder.getPosition(), ARM_DISCONTINUITY_RAD,
                ARM_DISCONTINUITY_RAD + 2 * Math.PI);
    }

    public double getWristPos() {
        return MathUtil.inputModulus(wristEncoder.getPosition(), ARM_DISCONTINUITY_RAD,
                ARM_DISCONTINUITY_RAD + 2 * Math.PI);
    }

    // Unbounded wrist position relative to ground
    public double getWristPosRelativeToGround() {
        return getArmPos() + wristEncoder.getPosition();
    }

    public void setArmTarget(double target) {
        goalPosRad[ARM] = getArmClampedGoal(target);
    }

    public void setWristTarget(double target) {
        goalPosRad[WRIST] = getWristClampedGoal(target);
    }

    public void setArmWristTarget(double targetArm, double targetWrist) {
        setArmTarget(targetArm);
        setWristTarget(targetWrist);
    }

    public boolean armAtSetpoint() {
        return armPID.atSetpoint();
    }

    public boolean wristAtSetpoint() {
        return wristPID.atSetpoint();
    }

    public boolean isCubeMode() {
        return object == CUBE;
    }

    public boolean isFrontMode() {
        return isFront;
    }

    public void toggleCube() {
        object++;
        object %= 2;
    }

    public void toggleFront() {
        isFront = !isFront;
    }

    public double getArmGoal(GoalPos[] pos) {
        if (isFront) {
            return pos[object].armPos;
        } else {
            return Rotation2d.fromRadians(-Math.PI).minus(Rotation2d.fromRadians(pos[object].armPos)).getRadians();
        }
    }

    public double getWristGoal(GoalPos[] pos) {
        if (isFront) {
            return pos[object].wristPos;
        } else {
            return Rotation2d.fromRadians(pos[object].wristPos).unaryMinus().getRadians();
        }
    }

    public void putCubePositionsOnSmartDashboard() {
        SmartDashboard.putNumber("LowArmCube", GoalPos.LOW[ARM].armPos);
        SmartDashboard.putNumber("LowWristCube", GoalPos.LOW[ARM].wristPos);
        SmartDashboard.putNumber("MidArmCube", GoalPos.MID[ARM].armPos);
        SmartDashboard.putNumber("MidWristCube", GoalPos.MID[ARM].wristPos);
        SmartDashboard.putNumber("HighArmCube", GoalPos.HIGH[ARM].armPos);
        SmartDashboard.putNumber("HighWristCube", GoalPos.HIGH[ARM].wristPos);
        SmartDashboard.putNumber("StoredArmCube", GoalPos.STORED[ARM].armPos);
        SmartDashboard.putNumber("StoredWristCube", GoalPos.STORED[ARM].wristPos);
        SmartDashboard.putNumber("ShelfArmCube", GoalPos.SHELF[ARM].armPos);
        SmartDashboard.putNumber("ShelfWristCube", GoalPos.SHELF[ARM].wristPos);
        SmartDashboard.putNumber("SubstationArmCube", GoalPos.SUBSTATION[ARM].armPos);
        SmartDashboard.putNumber("SubstationWristCube", GoalPos.SUBSTATION[ARM].wristPos);
        SmartDashboard.putNumber("IntakeArmCube", GoalPos.INTAKE[ARM].armPos);
        SmartDashboard.putNumber("IntakeWristCube", GoalPos.INTAKE[ARM].wristPos);
    }

    public void putConePositionsOnSmartDashboard() {
        SmartDashboard.putNumber("LowArmCone", GoalPos.LOW[WRIST].armPos);
        SmartDashboard.putNumber("LowWristCone", GoalPos.LOW[WRIST].wristPos);
        SmartDashboard.putNumber("MidArmCone", GoalPos.MID[WRIST].armPos);
        SmartDashboard.putNumber("MidWristCone", GoalPos.MID[WRIST].wristPos);
        SmartDashboard.putNumber("HighArmCone", GoalPos.HIGH[WRIST].armPos);
        SmartDashboard.putNumber("HighWristCone", GoalPos.HIGH[WRIST].wristPos);
        SmartDashboard.putNumber("StoredArmCone", GoalPos.STORED[WRIST].armPos);
        SmartDashboard.putNumber("StoredWristCone", GoalPos.STORED[WRIST].wristPos);
        SmartDashboard.putNumber("ShelfArmCone", GoalPos.SHELF[WRIST].armPos);
        SmartDashboard.putNumber("ShelfWristCone", GoalPos.SHELF[WRIST].wristPos);
        SmartDashboard.putNumber("SubstationArmCone", GoalPos.SUBSTATION[WRIST].armPos);
        SmartDashboard.putNumber("SubstationWristCone", GoalPos.SUBSTATION[WRIST].wristPos);
        SmartDashboard.putNumber("IntakeArmCone", GoalPos.INTAKE[WRIST].armPos);
        SmartDashboard.putNumber("IntakeWristCone", GoalPos.INTAKE[WRIST].wristPos);
    }

    public void getCubePositionsOnSmartDashboard() {
        GoalPos.LOW[CUBE].armPos = SmartDashboard.getNumber("LowArmCube", GoalPos.LOW[CUBE].armPos);
        GoalPos.LOW[CUBE].wristPos = SmartDashboard.getNumber("LowWristCube", GoalPos.LOW[CUBE].wristPos);
        GoalPos.MID[CUBE].armPos = SmartDashboard.getNumber("MidArmCube", GoalPos.MID[CUBE].armPos);
        GoalPos.MID[CUBE].wristPos = SmartDashboard.getNumber("MidWristCube", GoalPos.MID[CUBE].wristPos);
        GoalPos.HIGH[CUBE].armPos = SmartDashboard.getNumber("HighArmCube", GoalPos.HIGH[CUBE].armPos);
        GoalPos.HIGH[CUBE].wristPos = SmartDashboard.getNumber("HighWristCube", GoalPos.HIGH[CUBE].wristPos);
        GoalPos.STORED[CUBE].armPos = SmartDashboard.getNumber("StoredArmCube", GoalPos.STORED[CUBE].armPos);
        GoalPos.STORED[CUBE].wristPos = SmartDashboard.getNumber("StoredWristCube", GoalPos.STORED[CUBE].wristPos);
        GoalPos.SHELF[CUBE].armPos = SmartDashboard.getNumber("ShelfArmCube", GoalPos.SHELF[CUBE].armPos);
        GoalPos.SHELF[CUBE].wristPos = SmartDashboard.getNumber("ShelfWristCube", GoalPos.SHELF[CUBE].wristPos);
        GoalPos.SUBSTATION[CUBE].armPos = SmartDashboard.getNumber("SubstationArmCube",
                GoalPos.SUBSTATION[CUBE].armPos);
        GoalPos.SUBSTATION[CUBE].wristPos = SmartDashboard.getNumber("SubstationWristCube",
                GoalPos.SUBSTATION[CUBE].wristPos);
        GoalPos.INTAKE[CUBE].armPos = SmartDashboard.getNumber("IntakeArmCube", GoalPos.INTAKE[CUBE].armPos);
        GoalPos.INTAKE[CUBE].wristPos = SmartDashboard.getNumber("IntakeWristCube", GoalPos.INTAKE[CUBE].wristPos);
    }

    public void getConePositionsOnSmartDashboard() {
        GoalPos.LOW[CONE].armPos = SmartDashboard.getNumber("LowArmCone", GoalPos.LOW[CONE].armPos);
        GoalPos.LOW[CONE].wristPos = SmartDashboard.getNumber("LowWristCone", GoalPos.LOW[CONE].wristPos);
        GoalPos.MID[CONE].armPos = SmartDashboard.getNumber("MidArmCone", GoalPos.MID[CONE].armPos);
        GoalPos.MID[CONE].wristPos = SmartDashboard.getNumber("MidWristCone", GoalPos.MID[CONE].wristPos);
        GoalPos.HIGH[CONE].armPos = SmartDashboard.getNumber("HighArmCone", GoalPos.HIGH[CONE].armPos);
        GoalPos.HIGH[CONE].wristPos = SmartDashboard.getNumber("HighWristCone", GoalPos.HIGH[CONE].wristPos);
        GoalPos.STORED[CONE].armPos = SmartDashboard.getNumber("StoredArmCone", GoalPos.STORED[CONE].armPos);
        GoalPos.STORED[CONE].wristPos = SmartDashboard.getNumber("StoredWristCone", GoalPos.STORED[CONE].wristPos);
        GoalPos.SHELF[CONE].armPos = SmartDashboard.getNumber("ShelfArmCone", GoalPos.SHELF[CONE].armPos);
        GoalPos.SHELF[CONE].wristPos = SmartDashboard.getNumber("ShelfWristCone", GoalPos.SHELF[CONE].wristPos);
        GoalPos.SUBSTATION[CONE].armPos = SmartDashboard.getNumber("SubstationArmCone",
                GoalPos.SUBSTATION[CONE].armPos);
        GoalPos.SUBSTATION[CONE].wristPos = SmartDashboard.getNumber("SubstationWristCone",
                GoalPos.SUBSTATION[CONE].wristPos);
        GoalPos.INTAKE[CONE].armPos = SmartDashboard.getNumber("IntakeArmCone", GoalPos.INTAKE[CONE].armPos);
        GoalPos.INTAKE[CONE].wristPos = SmartDashboard.getNumber("IntakeWristCone", GoalPos.INTAKE[CONE].wristPos);
    }

    public void putPIDOnSmartDashboard() {
        SmartDashboard.putNumber("kP: Wis", kP[WRIST]);
        SmartDashboard.putNumber("kI: Wis", kI[WRIST]);
        SmartDashboard.putNumber("kD: Wis", kD[WRIST]);
        SmartDashboard.putNumber("kP: Arm", kP[ARM]);
        SmartDashboard.putNumber("kI: Arm", kI[ARM]);
        SmartDashboard.putNumber("kD: Arm", kD[ARM]);
    }

    public void getPIDOnSmartDashboard() {
        kP[WRIST] = SmartDashboard.getNumber("kP: Wis", kP[WRIST]);
        kI[WRIST] = SmartDashboard.getNumber("kI: Wis", kI[WRIST]);
        kD[WRIST] = SmartDashboard.getNumber("kD: Wis", kD[WRIST]);
        kP[ARM] = SmartDashboard.getNumber("kP: Arm", kP[ARM]);
        kI[ARM] = SmartDashboard.getNumber("kI: Arm", kI[ARM]);
        kD[ARM] = SmartDashboard.getNumber("kD: Arm", kD[ARM]);
    }

    public void putArmControlOnSmartDashboard() {
        SmartDashboard.putNumber("WristGoalDeg", Units.radiansToDegrees(goalPosRad[WRIST]));
        SmartDashboard.putNumber("ArmGoalDeg", Units.radiansToDegrees(goalPosRad[ARM]));
        SmartDashboard.putNumber("Max FF Vel Arm", MAX_FF_VEL[ARM]);
        SmartDashboard.putNumber("Max FF Accel Arm", MAX_FF_ACCEL[ARM]);
    }

    public void getArmControlOnSmartDashboard() {
        goalPosRad[WRIST] = Units
                .degreesToRadians(SmartDashboard.getNumber("WristGoalDeg", Units.radiansToDegrees(goalPosRad[WRIST])));
        goalPosRad[ARM] = Units
                .degreesToRadians(SmartDashboard.getNumber("ArmGoalDeg", Units.radiansToDegrees(goalPosRad[ARM])));
        MAX_FF_VEL[ARM] = SmartDashboard.getNumber("Max FF Vel Arm", MAX_FF_VEL[ARM]);
        MAX_FF_ACCEL[ARM] = SmartDashboard.getNumber("Max FF Accel Arm", MAX_FF_ACCEL[ARM]);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("WristGoalPos", () -> goalPosRad[1], goal -> goalPosRad[1] = goal);
        builder.addDoubleProperty("ArmPos", () -> getArmPos(), null);
        builder.addDoubleProperty("WisPos", () -> getWristPos(), null);
        builder.addDoubleProperty("RobotWisPos", () -> getWristPosRelativeToGround(), null);
        builder.addDoubleProperty("kP: Arm", () -> kP[ARM], x -> kP[ARM] = x);
        builder.addDoubleProperty("kP: Wis", () -> kP[WRIST], x -> kP[WRIST] = x);
        builder.addDoubleProperty("kI: Arm", () -> kI[ARM], x -> kI[ARM] = x);
        builder.addDoubleProperty("kI: Wis", () -> kI[WRIST], x -> kI[WRIST] = x);
        builder.addDoubleProperty("kD: Arm", () -> kD[ARM], x -> kD[ARM] = x);
        builder.addDoubleProperty("kD: Wis", () -> kD[WRIST], x -> kD[WRIST] = x);
    }
}