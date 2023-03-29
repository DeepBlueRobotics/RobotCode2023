package org.carlmontrobotics.robotcode2023.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class LoggingCommand extends WrapperCommand {

    public LoggingCommand(Command command) {
        super(command);
        //SmartDashboard.putBoolean(command.getName() + " Is Scheduled", command.isScheduled());
        //SmartDashboard.putBoolean(command.getName() + " Is Finished", command.isFinished());
    }

    @Override
    public void initialize() {
        super.initialize();
        System.out.println(m_command.getName() + " has been initalized");
    }

    @Override
    public void execute() {
        super.execute();
        SmartDashboard.putBoolean(m_command.getName() + " Is Scheduled", m_command.isScheduled());
        SmartDashboard.putBoolean(m_command.getName() + " Is Finished", m_command.isFinished());
        
    }
    
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (interrupted)
            System.out.println(m_command.getName() + " has been interrupted");
        else
            System.out.println(m_command.getName() + " has ended");
    }

    /*
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(m_command.getName());
        builder.addBooleanProperty("Is Scheduled", 
            () -> m_command.isScheduled(), 
            isSchedule -> {
                if (isSchedule)
                    CommandScheduler.getInstance().schedule(m_command);
                else
                    CommandScheduler.getInstance().cancel(m_command);
            }
        );
        builder.addBooleanProperty("Is Finished", 
            () -> m_command.isScheduled(), 
            null
        );
    }
    */
}