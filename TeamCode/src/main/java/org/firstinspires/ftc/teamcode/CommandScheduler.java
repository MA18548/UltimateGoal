package org.firstinspires.ftc.teamcode;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;

public class CommandScheduler {
    private static CommandScheduler commandScheduler;

    private HashMap<Subsystem, CommandBase> requirementsMap = new HashMap<Subsystem, CommandBase>();

    private CommandScheduler()
    {

    }

    public void schedule(CommandBase command)
    {
        for (Subsystem requiredSubsystem: command.getRequirements())
        {
            if (requirementsMap.containsKey(requiredSubsystem))
            {
                // TODO interrupt subsystem commands
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
        // run periodic functions
        for (Subsystem subsystem : requirementsMap.keySet())
        {
            subsystem.periodic();
        }

        for (Iterator<CommandBase> iterator = requirementsMap.values().iterator();
             iterator.hasNext(); ) {
            CommandBase command = iterator.next();

            command.execute();

            if (command.isFinished())
            {
                command.end(false);
                requirementsMap.keySet().removeAll(new ArrayList<Subsystem>(Arrays.asList(command.getRequirements())));
            }
        }
    }

    public void end(CommandBase command){
        command.end(true);
        requirementsMap.values().remove(command);
    }

    public bool isRunning(CommandBase command)
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
