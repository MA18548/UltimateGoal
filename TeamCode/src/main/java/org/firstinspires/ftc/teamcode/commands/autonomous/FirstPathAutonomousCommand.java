package org.firstinspires.ftc.teamcode.commands.autonomous;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.drivetrain.ShooterVisionCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterPIDCommand;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.ma.ftc.lib.command.CommandBase;
import org.ma.ftc.lib.command.CommandScheduler;
import org.ma.ftc.lib.command.RobotMap;
import org.ma.ftc.lib.control.MAPath;

public class FirstPathAutonomousCommand extends CommandBase {
    private int stage;
    private MAPath MAPath;
    private CommandBase shootingCommand;
    private CommandBase shooterVisionCommand;
    private final ElapsedTime timer;

    public FirstPathAutonomousCommand() {
        MAPath = new MAPath(0.2);
        shootingCommand = new ShooterPIDCommand(-2200);
        shooterVisionCommand = new ShooterVisionCommand();
        timer = RobotMap.getInstance().getRuntime();


        addRequirements(MecanumDriveSubsystem.getInstance(), ShooterSubsystem.getInstance(), CameraSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        stage = 0;
        MAPath.initialize();
        RobotMap.getInstance().getTelemtry().addData("I'm in init!", "");

    }

    @Override
    public void execute() {
        RobotMap.getInstance().getTelemtry().addData("I'm in execute!", "");
        switch (stage)
        {
            case 0:
                MAPath.execute();
                if(MAPath.isFinished()){
                    MAPath.end(true);
                    shooterVisionCommand.initialize();
                    stage++;
                }
                break;

            case 1:
                shooterVisionCommand.execute();
                if(shooterVisionCommand.isFinished()){
                    shooterVisionCommand.end(true);
                    shootingCommand.initialize();
                    stage++;
                }
                break;

            case 2:
                shootingCommand.execute();
                if(shootingCommand.isFinished()){
                    shootingCommand.end(true);
                }
                break;

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
