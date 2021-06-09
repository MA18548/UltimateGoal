package org.firstinspires.ftc.teamcode.commands.drivetrain;

import android.graphics.Camera;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.ma.ftc.lib.command.CommandBase;
import org.ma.ftc.lib.command.RobotMap;

public class ShooterVisionCommand extends CommandBase {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final double setpoint = 2000;

    public ShooterVisionCommand() {
        mecanumDriveSubsystem = MecanumDriveSubsystem.getInstance();

        addRequirements(mecanumDriveSubsystem);
    }

    @Override
    public void initialize() {
        mecanumDriveSubsystem.reset();
        mecanumDriveSubsystem.setAngleSetpoint(0);

        // driveTrainSubsystem.setDistanceSetpoint(setpoint);
    }

    @Override
    public void execute() {
        mecanumDriveSubsystem.arcadeDrive(-mecanumDriveSubsystem.getAngleVisionPID(CameraSubsystem.getInstance().vuforiaGetAngleTowerGoal()),
                0);
    }

    @Override
    public void end(boolean interrupted) {
        mecanumDriveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(CameraSubsystem.getInstance().vuforiaGetAngleTowerGoal()) <= 1.25;
    }
}
