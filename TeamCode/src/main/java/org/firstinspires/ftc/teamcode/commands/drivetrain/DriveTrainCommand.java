package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.ma.ftc.lib.command.CommandBase;
import org.ma.ftc.lib.command.MAGamepad;
import org.ma.ftc.lib.command.RobotMap;
import org.ma.ftc.lib.control.MecanumDrive;

public class DriveTrainCommand extends CommandBase {

    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final MAGamepad gamepad;

    public   DriveTrainCommand() {
        mecanumDriveSubsystem = MecanumDriveSubsystem.getInstance();
        gamepad = RobotMap.getInstance().getDrivingGamepad();

        addRequirements(mecanumDriveSubsystem);
    }


    public void initialize() {

    }

    public void execute() {
        double modifier = gamepad.gamepad.left_trigger > 0.5 ? 3 : 1;
        double rot_modifier = gamepad.gamepad.right_trigger > 0.5 ? 3 : 1;

        modifier = gamepad.gamepad.left_bumper ? .5 : modifier*1;
        rot_modifier = gamepad.gamepad.right_bumper ? .5 : rot_modifier*1;

        RobotMap.getInstance().getTelemtry().addData("distance:", mecanumDriveSubsystem.getAverageDistance());

        mecanumDriveSubsystem.driveCartesian(gamepad.getLeftY() * modifier, -gamepad.getLeftX() * modifier,
                                             gamepad.getRightX() * rot_modifier);
    }

    public void end(boolean interrupted) {
        mecanumDriveSubsystem.stop();
    }
}
