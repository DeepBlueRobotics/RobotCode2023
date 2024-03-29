package org.carlmontrobotics.robotcode2023.commands;

import org.carlmontrobotics.robotcode2023.Constants.Roller.GameObject;
import org.carlmontrobotics.robotcode2023.Constants.Roller.RollerMode;
import org.carlmontrobotics.robotcode2023.subsystems.Roller;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunRoller extends CommandBase {

    private final Roller roller;
    private final Timer timer = new Timer();
    private final RollerMode mode;

    public RunRoller(Roller roller, RollerMode mode) {
        addRequirements(this.roller = roller);
        this.mode = mode;
    }

    @Override
    public void initialize() {
        System.err.println("=============================RunRoller is Started=============================");

        if (mode.speed > 0) { // should not interrupt command to stop rollers
            if(roller.hasGamePiece() && isIntake()) cancel();
            if(!roller.hasGamePiece() && !isIntake()) cancel(); 
        }
        System.err.println("=============================RunRoller is initialized=============================");
        timer.reset();
        roller.setRollerMode(mode);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        // keep it running if its a cone
        if (mode.obj != GameObject.CONE && !interrupted)
            roller.setSpeed(0);
        timer.stop();
        System.err.println("=============================RunRoller has ended==========================================================");

    }

    @Override
    public boolean isFinished() {

        if(mode.speed == 0) return true;

        double time = timer.get();

        // TODO: distance sensor detects belt when wrist is spinning (concern)
        if (roller.hasGamePiece() == isIntake()) {
            timer.start();
        } else if (roller.hasGamePiece() != isIntake()) {
            timer.stop();
            timer.reset();
        }
        SmartDashboard.putString("Target Piece", mode.obj.toString());
        SmartDashboard.putNumber("Time Target", mode.time);
        SmartDashboard.putNumber("SetRoller Time Elapsed (s)", time);

        return roller.hasGamePiece() == isIntake() && time > mode.time;
    }

    // The rollerMode indicates what object it is trying to take in
    // if the object is NONE, then it is trying to outtake
    public boolean isIntake() {
        return (mode.obj != GameObject.NONE);
    }
}
