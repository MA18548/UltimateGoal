package org.ma.ftc.lib.command; // MA FTC 18548

/**
 * A state machine representing a complete action to be performed by the robot. Commands are run by
 * the {@link CommandScheduler}.
 *
 * <p>Commands are run synchronously from the main robot loop; no multithreading is used.
 */
public abstract class CommandBase {

    private SubsystemBase[] requiredSubsystem;
    private boolean isInterruptible = true;

    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     */
    public void initialize() {
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    public void execute() {
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally, or
     * when it interrupted/canceled.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    public void end(boolean interrupted) {
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    public boolean isFinished() {
        return false;
    }

    /**
     * Adds the specified requirements to the command.
     *
     * @param requirements the requirements to add
     */
    public void addRequirements(SubsystemBase... requirements) {
        requiredSubsystem = requirements;
    }

    /**
     * Schedules this command.
     *
     * @param interruptible whether this command can be interrupted by another command that shares one
     *                      of its requirements
     */
    public void schedule(boolean interruptible) {
        CommandScheduler.getInstance().schedule(this, interruptible);
    }

    /**
     * Schedules this command, defaulting to interruptible.
     */
    public void schedule() {
        this.schedule(true);
    }

    /**
     * Whether or not the command is currently scheduled. Note that this does not detect whether the
     * command is being run by a CommandGroup, only whether it is directly being run by the scheduler.
     *
     * @return Whether the command is scheduled.
     */
    public boolean isScheduled() {
        return CommandScheduler.getInstance().isScheduled(this);
    }

    /**
     * Sets whether the command is interruptible or not.
     *
     * @param interruptible whether this command can be interrupted by another command that shares one
     *                      of its requirements
     */
    public void setInterruptable(boolean interruptible) {
        isInterruptible = interruptible;
    }

    /**
     * Whether or not the command is interruptible or not .
     *
     * @return Whether the command is interruptible.
     */
    public boolean isInterruptible() {
        return isInterruptible;
    }

    /**
     * Cancels this command. Commands will be canceled
     * even if they are not marked as interruptible.
     */
    public void cancel() {
        CommandScheduler.getInstance().cancel(this);
    }

    /**
     * Sets this command as a default command
     */
    public void setAsDefaultCommand() {
        CommandScheduler.getInstance().setDefaultCommand(this);
    }

    /**
     * Specifies the array of subsystems used by this command. Two commands cannot use the same
     * subsystem at the same time. If the command is scheduled as interruptible and another command is
     * scheduled that shares a requirement, the command will be interrupted. Else, the command will
     * not be scheduled. If no subsystems are required, return an empty set.
     *
     * @return the array of subsystems that are required
     */
    public SubsystemBase[] getRequirements() {
        return requiredSubsystem;
    }
}
