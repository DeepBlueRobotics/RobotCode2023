package org.carlmontrobotics.robotcode2023.commands;

import java.awt.Color;

import org.carlmontrobotics.robotcode2023.subsystems.Roller;
import org.carlmontrobotics.robotcode2023.subsystems.Roller.RollerMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunRoller extends CommandBase {

    private final Roller roller;
    private final Color ledColor;
    private final Timer timer = new Timer();
    private final RollerMode mode;

    public RunRoller(Roller roller, RollerMode mode, Color ledColor) {
        addRequirements(this.roller = roller);
        this.mode = mode;
        this.ledColor = ledColor;
    }

    @Override
    public void initialize() {
        roller.setSpeed(mode.speed);
        roller.setLedColor(ledColor);
        timer.reset();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        roller.setSpeed(0);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        double time = timer.get();

        if (roller.hasGamePiece() == mode.intake) {
            timer.start();
        }
        SmartDashboard.putNumber("Time Target", mode.time);
        SmartDashboard.putNumber("SetRoller Time Elapsed (s)", time);

        return roller.hasGamePiece() == mode.intake && time > mode.time;
    }
}
