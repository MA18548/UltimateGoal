package org.firstinspires.ftc.teamcode;


public class exampleCommand extends CommandBase {

    private ExampleSubsystem exampleSubsystem;
    private double power;

    public exampleCommand(double power)
    {
        exampleSubsystem = ExampleSubsystem.getInstance();
        this.power = power;

        addRequirements(exampleSubsystem);
    }

    public void init()
    {

    }

    public void execute()
    {
        //exampleSubsystem.setMotor(this.power);
        BasicOpMode_Iterative.commandRunning = true;

    }

    public void end()
    {
        //exampleSubsystem.setMotor(0);
        BasicOpMode_Iterative.commandRunning = false;
    }
}
