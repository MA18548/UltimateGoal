package org.firstinspires.ftc.teamcode;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;

public class CommandScheduler {
    private static CommandScheduler commandScheduler;

    private HashMap<Subsystem, CommandBase> subsystemMap = new HashMap<Subsystem, CommandBase>();
    private HashMap<Subsystem, CommandBase> requirementsMap = new HashMap<Subsystem, CommandBase>();

    private CommandScheduler()
    {

    }

    public void schedule(CommandBase command)
    {
        for (Subsystem subsystem: command.getRequirements())
        {
            if (!subsystemMap.containsKey(key))
            {
                subsystemMap.put(subsystem, null);
            }
            if (requirementsMap.containsKey(subsystem))
            {
                this.end(command);
                return;
            }
        }

        for (Subsystem requiredSubsystem: command.getRequirements())
        {
            requirementsMap.put(requiredSubsystem, command);
        }

        command.init();
    }

    public void run()
    {
        for (Subsystem subsystem : subsystemMap.keySet())
        {
            subsystem.periodic();
            if (requirementsMap.get(subsystem))
            {
                CommandBase command = requirementsMap.get(subsystem);
                command.execute();

                if (command.isFinished())
                {
                    this.end(command);
                }
            }
            else if (subsystemMap.get(subsystem))
            {
                CommandBase command = subsystemMap.get(subsystem);
                command.execute();
            }
        }
    }

    public void end(CommandBase command)
    {
        command.end(true);
        requirementsMap.values().remove(command);
    }

    public void setDefaultCommand(CommandBase command)
    {
        for (Subsystem requiredSubsystem: command.getRequirements())
        {
            subsystemMap.replace(requiredSubsystem, command);
        }
    }

    public boolean isRunning(CommandBase command)
    {
        return requirementsMap.containsValue(command);
    }

    public static CommandScheduler getInstance()
    {
        if (commandScheduler == null)
        {
            commandScheduler = new CommandScheduler();
        }
        return commandScheduler;
    }
}
