package org.firstinspires.ftc.teamcode.commands.autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.drivetrain.ShooterVisionCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterPIDCommand;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.ma.ftc.lib.command.CommandBase;
import org.ma.ftc.lib.command.RobotMap;
import org.ma.ftc.lib.control.MAPath;
import org.ma.ftc.lib.control.Path;

public class StrafeRightPathAutonomousCommand extends CommandBase {
    private int stage;
    private MAPath MAPath;
    private CommandBase shootingCommand;
    private CommandBase shooterVisionCommand;
    private final ElapsedTime timer;
    private double time;
    private boolean isFinished;

    public StrafeRightPathAutonomousCommand() {
        MAPath = new MAPath(0.2);
        shootingCommand = new ShooterPIDCommand(-2200);
        shooterVisionCommand = new ShooterVisionCommand();
        timer = RobotMap.getInstance().getRuntime();
        time = timer.seconds();
        isFinished = false;
        Path.mainPath = Path.strafeRight;
        addRequirements(MecanumDriveSubsystem.getInstance(), ShooterSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        stage = 0;
        ShooterSubsystem.getInstance().VISION = true;
        MAPath.initialize();
        RobotMap.getInstance().getTelemtry().addData("I'm in init!", "");

    }

    @Override
    public void execute() {
        RobotMap.getInstance().getTelemtry().addData("I'm in autonomous!", "");
        RobotMap.getInstance().getTelemtry().addData("Autonomous stage: ", stage);

        switch (stage) {
            case 0:
                MAPath.execute();
                if (MAPath.isFinished()) {
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
                    time = timer.seconds();
                }
                break;

            case 2:
                shootingCommand.execute();
                if(timer.seconds() - time >= 3.5){
                    shootingCommand.end(true);
                    time = timer.seconds();
                    stage++;

                }
                break;
            case 3:
                MecanumDriveSubsystem.getInstance().arcadeDrive(0, 0.65);
                if(timer.seconds() - time >= 0.77){
                    MecanumDriveSubsystem.getInstance().stop();
                    stage++;
                    isFinished = true;

                }
                break;
        }
        }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
