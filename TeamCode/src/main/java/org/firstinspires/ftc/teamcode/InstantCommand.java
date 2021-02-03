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
    m_toRun = () -> {};
  }

  @Override
  public void initialize() 
  {
    toRun.run();
  }

  @Override
  public boolean isFinished() 
  {
    return true;
  }
}