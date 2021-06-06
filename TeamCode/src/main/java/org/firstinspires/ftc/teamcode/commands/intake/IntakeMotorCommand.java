package org.firstinspires.ftc.teamcode.commands.intake;

import org.firstinspires.ftc.teamcode.ExampleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.ma.ftc.lib.command.CommandBase;

public class IntakeMotorCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;
    private double power;

    public IntakeMotorCommand(double power)
    {
        intakeSubsystem = IntakeSubsystem.getInstance();
        this.power = power;

        addRequirements(intakeSubsystem);
    }

    public void init()
    {

    }

    public void execute()
    {
        intakeSubsystem.setMotor(this.power);
    }

    public void end(boolean interrupted)
    {
        intakeSubsystem.setMotor(0);
    }
}
