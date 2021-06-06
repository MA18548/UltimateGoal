package org.firstinspires.ftc.teamcode.commands.drivetrain;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.ma.ftc.lib.command.CommandBase;

public class DriveTrainInterrupt extends CommandBase {

    private final DriveTrainSubsystem exampleSubsystem;

    public DriveTrainInterrupt() {
        exampleSubsystem = DriveTrainSubsystem.getInstance();

        addRequirements(exampleSubsystem);
    }

    public void initialize() {

    }

    public void execute() {
        exampleSubsystem.tankDrive(.5, .5);
    }

    public void end(boolean interrupted) {
        exampleSubsystem.tankDrive(0, 0);
    }
}
