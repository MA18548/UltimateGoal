package org.firstinspires.ftc.teamcode.commands.drivetrain;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.ma.ftc.lib.command.CommandBase;
import org.ma.ftc.lib.command.RobotMap;

public class PID extends CommandBase {
    private final MecanumDriveSubsystem driveTrainSubsystem;
    private final double setpoint = 2000;

    public PID() {
        driveTrainSubsystem = MecanumDriveSubsystem.getInstance();

        addRequirements(driveTrainSubsystem);
    }

    @Override
    public void initialize() {
        driveTrainSubsystem.reset();
        driveTrainSubsystem.setAngleSetpoint(0);
        driveTrainSubsystem.setDistanceSetpoint(1750);
        // driveTrainSubsystem.setDistanceSetpoint(setpoint);
    }

    @Override
    public void execute() {
        driveTrainSubsystem.powerCalc(-driveTrainSubsystem.getDistancePID(), 0, driveTrainSubsystem.getAnglePID());
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
