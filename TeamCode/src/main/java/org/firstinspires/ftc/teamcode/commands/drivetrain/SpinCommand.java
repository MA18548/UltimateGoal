package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.ma.ftc.lib.command.CommandBase;
import org.ma.ftc.lib.command.RobotMap;

public class SpinCommand extends CommandBase {
    private final MecanumDriveSubsystem driveTrainSubsystem;

    public SpinCommand() {
        driveTrainSubsystem = MecanumDriveSubsystem.getInstance();

        addRequirements(driveTrainSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        driveTrainSubsystem.arcadeDrive(1, 0);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrainSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
