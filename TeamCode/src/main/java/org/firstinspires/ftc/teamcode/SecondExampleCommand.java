package org.firstinspires.ftc.teamcode;

public class SecondExampleCommand extends CommandBase {

    private ExampleSubsystem exampleSubsystem;

    public SecondExampleCommand()
    {
        exampleSubsystem = ExampleSubsystem.getInstance();

        addRequirements(exampleSubsystem);
    }

    public void init()
    {
    }
}
