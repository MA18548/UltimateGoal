package org.firstinspires.ftc.teamcode.commands.intake;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.ma.ftc.lib.command.CommandBase;
import org.ma.ftc.lib.command.RobotMap;

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
        RobotMap.getInstance().getTelemtry().addData("Intake Speed: ", intakeSubsystem.getSpeed());
        intakeSubsystem.setMotor(this.power);
    }

    public void end(boolean interrupted)
    {
        RobotMap.getInstance().getTelemtry().addData("I'm in end!", "");
        intakeSubsystem.setMotor(0);
    }
}
