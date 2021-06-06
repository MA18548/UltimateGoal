package org.ma.ftc.lib.command; // MA FTC 18548


import org.ma.ftc.lib.Runnable;

public class InstantCommand extends CommandBase {
    private final Runnable toRun;

    public InstantCommand(Runnable toRun, SubsystemBase... requirements) {
        this.toRun = toRun;

        addRequirements(requirements);
    }

    public InstantCommand() {
        toRun = new Runnable();
    }

    public void initialize() {
        toRun.run();
    }

    public boolean isFinished() {
        return true;
    }
}