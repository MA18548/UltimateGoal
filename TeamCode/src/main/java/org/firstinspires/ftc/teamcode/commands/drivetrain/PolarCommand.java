package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.ma.ftc.lib.command.CommandBase;
import org.ma.ftc.lib.command.CommandScheduler;
import org.ma.ftc.lib.command.RobotMap;

public class PolarCommand extends CommandBase {
    private final MecanumDriveSubsystem mecanumSubsystem;

    public PolarCommand() {
        mecanumSubsystem = MecanumDriveSubsystem.getInstance();

        addRequirements(mecanumSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        mecanumSubsystem.drivePolar(1, Math.toRadians(60), 0.1);
    }

    @Override
    public void end(boolean interrupted) {
        mecanumSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
