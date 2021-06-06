package org.ma.ftc.lib.command; // MA FTC 18548

import org.ma.ftc.lib.Runnable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

/**
 * The scheduler responsible for running {@link CommandBase}s. A Command-based robot should call {@link
 * CommandScheduler#run()} on the singleton instance in its periodic block in order to run commands
 * synchronously from the main loop. Subsystems should be registered with the scheduler using {@link
 * CommandScheduler#registerSubsystem(SubsystemBase...)} in order for their {@link SubsystemBase#periodic()}
 * methods to be called and for their default commands to be scheduled.
 */

public class CommandScheduler {
    /**
     * The Singleton Instance.
     */
    private static CommandScheduler commandScheduler;

    // A map from subsystems registered with the scheduler to their default commands.  Also used
    // as a list of currently-registered subsystems.
    private final HashMap<SubsystemBase, CommandBase> subsystemMap = new HashMap<SubsystemBase, CommandBase>();

    // A map from subsystems to their current running commands.
    private final HashMap<SubsystemBase, CommandBase> requirementsMap = new HashMap<SubsystemBase, CommandBase>();

    // The list of currently-registered buttons that will be polled every iteration.
    private final ArrayList<Runnable> buttons = new ArrayList<Runnable>();

    private CommandScheduler() {

    }

    @Override
    public String toString() {
        return "CommandScheduler:" +
                "\tsubsystemMap - " + subsystemMap +
                ",\trequirementsMap - " + requirementsMap +
                ",\tbuttons - " + buttons;
    }

    /**
     * Schedules a command for execution. Does nothing if the command is already scheduled. If a
     * command's requirements are not available, it will only be started if all the commands currently
     * using those requirements have been scheduled as interruptible. If this is the case, they will
     * be interrupted and the command will be scheduled.
     *
     * @param interruptible whether this command can be interrupted
     * @param command       the command to schedule
     */

    public void schedule(CommandBase command, boolean interruptible) {
        for (SubsystemBase subsystem : command.getRequirements()) {
            if (requirementsMap.containsKey(subsystem)) {
                if (!(requirementsMap.get(subsystem).isInterruptible()
                        || requirementsMap.get(subsystem).equals(command))) {
                    return;
                }
            }
        }

        for (SubsystemBase requiredSubsystem : command.getRequirements()) {
            if (requirementsMap.containsKey(requiredSubsystem)) {
                requirementsMap.get(requiredSubsystem).cancel();
            }
            requirementsMap.put(requiredSubsystem, command);
        }

        command.setInterruptable(interruptible);
        command.initialize();
    }

    /**
     * Schedules a command for execution. Does nothing if the command is already scheduled. If a
     * command's requirements are not available, it will only be started if all the commands currently
     * using those requirements have been scheduled as interruptible. If this is the case, they will
     * be interrupted and the command will be scheduled.
     *
     * @param command the command to schedule
     */
    public void schedule(CommandBase command) {
        schedule(command, true);
    }

    /**
     * Schedules multiple commands for execution. Does nothing if the command is already scheduled. If
     * a command's requirements are not available, it will only be started if all the commands
     * currently using those requirements have been scheduled as interruptible. If this is the case,
     * they will be interrupted and the command will be scheduled.
     *
     * @param interruptible whether the commands should be interruptible
     * @param commands      the commands to schedule
     */
    public void schedule(boolean interruptible, CommandBase... commands) {
        for (CommandBase command : commands) {
            schedule(interruptible, command);
        }
    }

    /**
     * Schedules multiple commands for execution, with interruptible defaulted to true. Does nothing
     * if the command is already scheduled.
     *
     * @param commands the commands to schedule
     */
    public void schedule(CommandBase... commands) {
        schedule(true, commands);
    }

    /**
     * Runs a single iteration of the scheduler. The execution occurs in the following order:
     *
     * <p>Subsystem periodic methods are called.
     *
     * <p>Button bindings are polled, and new commands are scheduled from them.
     *
     * <p>Currently-scheduled commands are executed.
     *
     * <p>End conditions are checked on currently-scheduled commands, and commands that are finished
     * have their end methods called and are removed.
     *
     * <p>Any subsystems not being used as requirements have their default methods started.
     */
    public void run() {
        for (Runnable button : buttons) {
            button.run();
        }

        for (SubsystemBase subsystem : subsystemMap.keySet()) {
            subsystem.periodic();
            if (requirementsMap.containsKey(subsystem)) {
                CommandBase command = requirementsMap.get(subsystem);
                command.execute();

                if (command.isFinished()) {
                    this.end(command, false);
                }
            }
        }
    }

    /**
     * Ends a certain command.
     *
     * @param command     the command to end
     * @param interrupted whether the command interrupted or not
     */
    public void end(CommandBase command, boolean interrupted) {
        command.end(interrupted);
        requirementsMap.values().remove(command);

        for (SubsystemBase subsystem : command.getRequirements()) {
            if (subsystemMap.containsKey(subsystem)) {
                CommandBase defaultCommand = subsystemMap.get(subsystem);
                if (defaultCommand != null && !defaultCommand.equals(command)) {
                    defaultCommand.schedule();
                }
            }
        }
    }

    /**
     * Ends a certain command with an interrupt.
     *
     * @param command the command to end
     */
    public void cancel(CommandBase command) {
        end(command, true);
    }

    /**
     * Stops all running commands. Important thing to note that default commands will be rescheduled
     */

    public void clearCommands() {
        requirementsMap.clear();
    }

    public void clearSubsystems()
    {
        subsystemMap.clear();
    }
    /**
     * Adds a button binding to the scheduler, which will be polled to schedule commands.
     *
     * @param button The button to add
     */
    public void addButton(Runnable button) {
        buttons.add(button);
    }

    /**
     * Removes all button bindings from the scheduler.
     */
    public void clearButtons() {
        buttons.clear();
    }

    /**
     * Registers subsystems with the scheduler. This must be called for the subsystem's periodic block
     * to run when the scheduler is run, and for the subsystem's default command to be scheduled. It
     * is recommended to call this from the constructor of your subsystem implementations.
     *
     * @param subsystems the subsystem to register
     */
    public void registerSubsystem(SubsystemBase... subsystems) {
        for (SubsystemBase subsystem : subsystems) {
            subsystemMap.put(subsystem, null);
        }
    }

    /**
     * Un-registers subsystems with the scheduler. The subsystem will no longer have its periodic
     * block called, and will not have its default command scheduled.
     *
     * @param subsystems the subsystem to un-register
     */
    public void unregisterSubsystem(SubsystemBase... subsystems) {
        subsystemMap.values().removeAll(Arrays.asList(subsystems));
    }


    /**
     * Sets the default command for it's required subsystems.
     * Default commands will run whenever there is no other command currently scheduled
     * that requires the subsystem. Default commands should be written to never end (i.e. their {@link
     * CommandBase#isFinished()} method should return false), as they would simply be re-scheduled if they
     * do. Default commands must also require their subsystems.
     *
     * @param command the default command
     */
    public void setDefaultCommand(CommandBase command) {
        command.schedule();
        for (SubsystemBase requiredSubsystem : command.getRequirements()) {
            subsystemMap.put(requiredSubsystem, command);
        }
    }

    /**
     * Gets the default command associated with this subsystem. Null if this subsystem has no default
     * command associated with it.
     *
     * @param subsystem the subsystem to inquire about
     * @return the default command associated with the subsystem
     */
    public CommandBase getDefaultCommand(SubsystemBase subsystem) {
        return subsystemMap.get(subsystem);
    }

    /**
     * Whether the given commands are running. Note that this only works on commands that are directly
     * scheduled by the scheduler;
     *
     * @param commands the command to query
     * @return whether the command is currently scheduled
     */

    public boolean isScheduled(CommandBase... commands) {
//        return requirementsMap.keySet().containsAll(Set.of(commands));
        return true;
    }


    /**
     * Returns the Scheduler instance.
     *
     * @return the instance
     */
    public static CommandScheduler getInstance() {
        if (commandScheduler == null) {
            commandScheduler = new CommandScheduler();
        }
        return commandScheduler;
    }
}
