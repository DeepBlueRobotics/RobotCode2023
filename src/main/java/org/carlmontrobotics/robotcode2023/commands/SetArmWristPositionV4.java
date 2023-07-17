package org.carlmontrobotics.robotcode2023.commands;

import static org.carlmontrobotics.robotcode2023.Constants.Arm.*;

import org.carlmontrobotics.robotcode2023.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SetArmWristPositionV4 extends ProxyCommand {

    public String name;

    public SetArmWristPositionV4(double targetArmPos, double targetWristPos, Arm arm) {
        super(() -> {
            double currentArmPos = arm.getArmPos();
            double currentWristPos = arm.getWristPos();

            if(Arm.isWristOutsideRobot(currentArmPos, currentWristPos)) {
                if(Arm.isWristOutsideRobot(targetArmPos, targetWristPos)) {
                    if(Math.signum(currentArmPos - ARM_VERTICAL_POS_RAD) == Math.signum(targetArmPos - ARM_VERTICAL_POS_RAD)) {
                        return moveSidePosToSameSide(currentArmPos, currentWristPos, targetArmPos, targetWristPos, arm);
                    } else {
                        return moveSidePosToOtherSide(currentArmPos, currentWristPos, targetArmPos, targetWristPos, arm);
                    }
                } else {
                    return moveSidePosIntoRobot(currentArmPos, currentWristPos, targetArmPos, targetWristPos, arm);
                }
            } else {
                if(Arm.isWristOutsideRobot(targetArmPos, targetWristPos)) {
                    return moveRobotPosToSide(currentArmPos, currentWristPos, targetArmPos, targetWristPos, arm);
                } else {
                    return moveRobotPosIntoRobot(currentArmPos, currentWristPos, targetArmPos, targetWristPos, arm);
                }
            }
        });

        name = "SetArmWristPositionV3(" + targetArmPos + ", " + targetWristPos + ")";

        addRequirements(arm);
    }

    @Override
    public String getName() {
        return name;
    }


    //#region Main Logic Paths

    public static Command moveSidePosToSameSide(double currentArmPos, double currentWristPos, double targetArmPos, double targetWristPos, Arm arm) {
        return new ParallelCommandGroup(
            moveArm(targetArmPos, arm),
            moveWrist(targetWristPos, arm)
        );
    }

    public static Command moveSidePosIntoRobot(double currentArmPos, double currentWristPos, double targetArmPos, double targetWristPos, Arm arm) {
        return new SequentialCommandGroup(
            parallelFoldWrist(currentArmPos, currentWristPos, targetWristPos, arm),
            moveArm(targetArmPos, arm),
            moveWrist(targetWristPos, arm)
        );
    }

    public static Command moveSidePosToOtherSide(double currentArmPos, double currentWristPos, double targetArmPos, double targetWristPos, Arm arm) {
        return new SequentialCommandGroup(
            parallelFoldWrist(currentArmPos, currentWristPos, targetWristPos, arm),
            safelyUnfoldWrist(currentArmPos, targetArmPos, targetWristPos, arm)
        );
    }

    public static Command moveRobotPosIntoRobot(double currentArmPos, double currentWristPos, double targetArmPos, double targetWristPos, Arm arm) {
        if(Math.signum(currentWristPos) == Math.signum(targetWristPos)) {
            return new SequentialCommandGroup(
                foldWristToSide(currentWristPos, arm),
                moveArm(targetArmPos, arm),
                moveWrist(targetWristPos, arm)
            );
        } else {
            return new SequentialCommandGroup(
                foldWristToSide(currentWristPos, arm),
                moveArm(ARM_VERTICAL_POS_RAD - MIN_WRIST_FOLD_POS_RAD, arm),
                foldWristToSide(targetWristPos, arm),
                moveArm(targetArmPos, arm),
                moveWrist(targetWristPos, arm)
            );
        }
    }

    public static Command moveRobotPosToSide(double currentArmPos, double currentWristPos, double targetArmPos, double targetWristPos, Arm arm) {
        if(Math.signum(currentWristPos) == Math.signum(targetWristPos) || Arm.canSafelyMoveWrist(targetArmPos)) {
            return new SequentialCommandGroup(
                foldWristToSide(currentWristPos, arm),
                safelyUnfoldWrist(currentArmPos, targetArmPos, targetWristPos, arm)
            );
        } else {
            return new SequentialCommandGroup(
                foldWristToSide(currentWristPos, arm),
                moveArmToSidedSafeWristMovePosition(targetArmPos, arm),
                moveWrist(targetWristPos, arm),
                moveArm(targetArmPos, arm)
            );
        }
    }

    //#endregion

    //#region Helper Methods

    public static Command moveArm(double pos, Arm arm) {
        return new SequentialCommandGroup(new InstantCommand(() -> arm.setArmTarget(pos, 0)), new WaitUntilCommand(arm::armAtSetpoint));
    }

    public static Command moveWrist(double pos, Arm arm) {
        return new SequentialCommandGroup(new InstantCommand(() -> arm.setWristTarget(pos, 0)), new WaitUntilCommand(arm::wristAtSetpoint));
    }

    // Moves the arm to a position where the wrist can safely be moved on the side of the provided arm position
    public static Command moveArmToSidedSafeWristMovePosition(double pos, Arm arm) {
        return moveArm(ARM_VERTICAL_POS_RAD + Math.copySign(MIN_WRIST_FOLD_POS_RAD, pos - ARM_VERTICAL_POS_RAD), arm);
    }

    // Folds the wrist to the same side as the provided position
    public static Command foldWristToSide(double pos, Arm arm) {
        return moveWrist(pos >= 0 ? WRIST_STOW_POS_RAD : WRIST_NEG_STOW_POS_RAD, arm);
    }

    public static Command ensureCanMoveWrist(double armPos, Arm arm) {
        return Arm.canSafelyMoveWrist(armPos) ? new InstantCommand() : moveArmToSidedSafeWristMovePosition(armPos, arm);
    }

    // Folds the wrist while moving the arm if possible
    public static Command parallelFoldWrist(double currentArmPos, double currentWristPos, double targetWristPos, Arm arm) {
        if(Math.signum(currentWristPos) == Math.signum(targetWristPos) || Arm.canSafelyMoveWrist(currentArmPos)) {
            return new ParallelCommandGroup(
                foldWristToSide(targetWristPos, arm),
                Arm.canSafelyMoveWrist(currentArmPos) ? moveArmToSidedSafeWristMovePosition(currentArmPos, arm) : new InstantCommand()
            );
        } else {
            return new SequentialCommandGroup(
                moveArmToSidedSafeWristMovePosition(currentArmPos, arm),
                foldWristToSide(targetWristPos, arm)
            );
        }
    }

    // Assumes that no additional arm movement is needed past the arm target position to move the wrist
    public static Command safelyUnfoldWrist(double currentArmPos, double targetArmPos, double targetWristPos, Arm arm) {
        return new ParallelCommandGroup(
            moveArm(targetArmPos, arm),
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> {
                    double armPos = arm.getArmPos();
                    return (Math.signum(ARM_VERTICAL_POS_RAD - armPos) == Math.signum(ARM_VERTICAL_POS_RAD - targetArmPos) && Arm.canSafelyMoveWrist(armPos)) || arm.armAtSetpoint();
                }),
                moveWrist(targetWristPos, arm)
            )
        );
    }

    //#endregion

}
