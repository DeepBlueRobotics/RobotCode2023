package org.carlmontrobotics.robotcode2023.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class CommandLog extends WrapperCommand {

    public CommandLog(Command command) {
        super(command);
        SmartDashboard.putBoolean(command.getName() + " Is Scheduled", command.isScheduled());
        SmartDashboard.putBoolean(command.getName() + " Is Finished", command.isFinished());
    }

    @Override
    public void execute() {
        super.execute();
        SmartDashboard.putBoolean(m_command.getName() + " Is Scheduled", m_command.isScheduled());
        SmartDashboard.putBoolean(m_command.getName() + " Is Finished", m_command.isFinished());
    }

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
}