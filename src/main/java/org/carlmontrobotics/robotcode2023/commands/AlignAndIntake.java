package org.carlmontrobotics.robotcode2023.commands;

import org.carlmontrobotics.lib199.Limelight;
import org.carlmontrobotics.robotcode2023.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignAndIntake extends CommandBase {

    private final Drivetrain drivetrain;
    private final Limelight lime;
    private final TeleopDrive teleopDrive;

    public AlignAndIntake(Drivetrain drivetrain, Limelight lime) {
        addRequirements(this.drivetrain = drivetrain);
        this.lime = lime;
        teleopDrive = (TeleopDrive) drivetrain.getDefaultCommand();
    }

    @Override
    public void execute() {
        double[] requestedSpeeds = teleopDrive.getRequestedSpeeds();

        drivetrain.drive(requestedSpeeds[0], requestedSpeeds[1], lime.steeringAssist());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
