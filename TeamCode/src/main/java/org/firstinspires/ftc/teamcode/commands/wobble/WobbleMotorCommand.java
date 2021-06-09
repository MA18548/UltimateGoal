package org.firstinspires.ftc.teamcode.commands.wobble;

import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSubsystem;
import org.ma.ftc.lib.command.CommandBase;
import org.ma.ftc.lib.command.RobotMap;

public class WobbleMotorCommand extends CommandBase {
    private final WobbleSubsystem wobbleSubsystem;

    public WobbleMotorCommand() {
        wobbleSubsystem = WobbleSubsystem.getInstance();

        addRequirements(wobbleSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
            wobbleSubsystem.setMotor(RobotMap.getInstance().getSystemGamepad().getLeftY());
    }

    @Override
    public void end(boolean interrupted) {
        wobbleSubsystem.setMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
