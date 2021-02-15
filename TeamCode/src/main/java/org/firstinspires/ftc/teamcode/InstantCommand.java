package org.firstinspires.ftc.teamcode;


public class InstantCommand extends CommandBase {
  private Runnable toRun;

  public InstantCommand(Runnable toRun, Subsystem... requirements) 
  {
    this.toRun = toRun;

    addRequirements(requirements);
  }

  public InstantCommand() 
  {
    toRun = () -> {};
  }

  public void initialize()
  {
    toRun.run();
  }

  public boolean isFinished()
  {
    return true;
  }
}