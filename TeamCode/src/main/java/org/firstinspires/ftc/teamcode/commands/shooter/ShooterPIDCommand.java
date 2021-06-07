package org.firstinspires.ftc.teamcode.commands.shooter;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.ma.ftc.lib.command.CommandBase;
import org.ma.ftc.lib.command.RobotMap;

public class ShooterPIDCommand extends CommandBase {

    private ShooterSubsystem shooterSubsystem;
    private ElapsedTime timer;
    private double rpmSetpoint;
    private double shootTime;
    private boolean isReset;

    public ShooterPIDCommand(double setpoint)
    {
        shooterSubsystem = ShooterSubsystem.getInstance();
        timer = RobotMap.getInstance().getRuntime();
        rpmSetpoint = setpoint;
        isReset = true;
        addRequirements(shooterSubsystem);
    }


    public void initialize() {
        double distance = CameraSubsystem.getInstance().vuforiaGetDistanceTowerGoal();
        shootTime = timer.seconds();
        shooterSubsystem.setSetpoint(shooterSubsystem.SETPOINT);
        if (shooterSubsystem.VISION)
        {
            distance = distance == 0 ? 5000 : distance;
            shooterSubsystem.setSetpoint(shooterSubsystem.getRPMByDistance(distance));
        }
        RobotMap.getInstance().getTelemtry().addData("I'm in init", "");
        shooterSubsystem.resetServo();

    }
    public void execute()
    {
        RobotMap.getInstance().getTelemtry().addData("Shooter RPM: ", shooterSubsystem.getRPM());
        RobotMap.getInstance().getTelemtry().addData("Shooter Setpoint: ", shooterSubsystem.SETPOINT);
        RobotMap.getInstance().getTelemtry().addData("Shooter at setpoint: ", shooterSubsystem.atSetpoint());


//        RobotMap.getInstance().getTelemtry().addData("Shooter Servo: ", shooterSubsystem.getServo());

        shooterSubsystem.runRPM();
        if(isReset && shooterSubsystem.getIR()
                && shooterSubsystem.atSetpoint() && timer.seconds() - shootTime > 0.4){

            shooterSubsystem.loadToFlywheel();
            shootTime = timer.seconds();
            isReset = false;
        }
        else if (!isReset && timer.seconds() - shootTime > 0.75
        )
        {
            shootTime = timer.seconds();
            shooterSubsystem.resetServo();
            isReset = true;
        }
    }

    public void end(boolean interrupted)
    {
        shooterSubsystem.stopMotor();
        RobotMap.getInstance().getTelemtry().addData("I'm in end", "");

        shooterSubsystem.resetServo();
    }

    public boolean isFinished()
    {

        return false;
    }
}
