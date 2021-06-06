package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.ma.ftc.lib.command.CommandBase;
import org.ma.ftc.lib.command.RobotMap;

public class ArcadeDriveCommand extends CommandBase {
    private final MecanumDriveSubsystem driveTrainSubsystem;

    public ArcadeDriveCommand() {
        driveTrainSubsystem = MecanumDriveSubsystem.getInstance();

        addRequirements(driveTrainSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
//        driveTrainSubsystem.arcadeDrive(RobotMap.getInstance().getGamepad().getLeftX(),
//                RobotMap.getInstance().getGamepad().getLeftY());
        //driveTrainSubsystem.powerCalc(-RobotMap.getInstance().getGamepad().getLeftY(),
              //  -RobotMap.getInstance().getGamepad().getLeftX(), RobotMap.getInstance().getGamepad().getRightX());
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
