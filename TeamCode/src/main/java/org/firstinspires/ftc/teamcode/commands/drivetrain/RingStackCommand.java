package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.ma.ftc.lib.command.CommandBase;
import org.ma.ftc.lib.command.RobotMap;

import java.util.List;

public class RingStackCommand extends CommandBase {
    private final MecanumDriveSubsystem driveTrainSubsystem;

    public RingStackCommand() {
        driveTrainSubsystem = MecanumDriveSubsystem.getInstance();

        addRequirements(driveTrainSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        RobotMap.getInstance().getTelemtry().addData("Rings found:", "");
        List<Recognition> recognitions = CameraSubsystem.getInstance().getTFOBObjects();
        for (Recognition rec : recognitions)
        {
            RobotMap.getInstance().getTelemtry().addData("Recognition: ", rec.getLabel());
        }

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
