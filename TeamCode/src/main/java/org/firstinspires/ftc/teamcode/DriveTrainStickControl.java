package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.ma.ftc.lib.command.CommandBase;
import org.ma.ftc.lib.command.RobotMap;

public class DriveTrainStickControl extends CommandBase {

    private DriveTrainSubsystem exampleSubsystem;

    public DriveTrainStickControl()
    {
        exampleSubsystem = DriveTrainSubsystem.getInstance();

        addRequirements(exampleSubsystem);
    }

    public void init()
    {
    }


    public void execute()
    {
        //exampleSubsystem.tankDrive(RobotMap.getInstance().getGamepad().getLeftY(),
             //   -RobotMap.getInstance().getGamepad().getRightY());
    }

    public void end(boolean interrupted)
    {
        exampleSubsystem.tankDrive(0, 0);
    }
}