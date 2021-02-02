package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public abstract class CommandBase {

    private Subsystem[] requiredSubsystem;

    void init() {}
    void execute() {}
    void end(boolean interrupted)  {}

    public boolean isFinished()
    {
        return false;
    }

    public void addRequirements(Subsystem... subsystems)
    {
        requiredSubsystem = subsystems;
    }

    public void schedule()
    {
        CommandScheduler.getInstance().schedule(this);
    }

    public boolean isRunning()
    {
        return CommandScheduler.getInstance().isRunning(this);
    }

    public void setAsDefaultCommand()
    {
        CommandScheduler.getInstance().setDefaultCommand(this);
    }

    Subsystem[] getRequirements() { return requiredSubsystem; }
}
