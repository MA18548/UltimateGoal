package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public abstract class CommandBase {

    private Subsystem[] requiredSubsystem;

    void init() {}
    void execute() {}
    void end(boolean interrupted)  {}

    boolean isFinished()
    {
        return false;
    }

    void addRequirements(Subsystem... subsystems)
    {
        requiredSubsystem = subsystems;
    }

    boolean isRunning()
    {
        return CommandScheduler.getInstance().isRunning(this);
    }

    Subsystem[] getRequirements() { return requiredSubsystem; }
}
