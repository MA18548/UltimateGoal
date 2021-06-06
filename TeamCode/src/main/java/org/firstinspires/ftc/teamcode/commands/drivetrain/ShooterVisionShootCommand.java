//package org.firstinspires.ftc.teamcode.commands.drivetrain;
//
//import android.graphics.Camera;
//
//import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
//import org.ma.ftc.lib.command.CommandBase;
//import org.ma.ftc.lib.command.RobotMap;
//import org.ma.ftc.lib.control.Movement;
//
//public class ShooterVisionShootCommand extends CommandBase {
//    private final MecanumDriveSubsystem mecanumDriveSubsystem;
//    private final ShooterSubsystem shooterSubsystem;
//    private final double setpoint = 2000;
//
//    private boolean isReady;
//    private boolean isReset;
//
//
//    public ShooterVisionShootCommand() {
//        mecanumDriveSubsystem = MecanumDriveSubsystem.getInstance();
//        shooterSubsystem = ShooterSubsystem.getInstance();
//        isReady = false;
//        isReset = true;
//        addRequirements(mecanumDriveSubsystem, shooterSubsystem);
//    }
//
//    @Override
//    public void initialize() {
//        mecanumDriveSubsystem.reset();
//        mecanumDriveSubsystem.setAngleSetpoint(0);
//        mecanumDriveSubsystem.setDistanceSetpoint(0);
//        // driveTrainSubsystem.setDistanceSetpoint(setpoint);
//    }
//
//    @Override
//    public void execute() {
//        if (!isReady)
//        {
//            mecanumDriveSubsystem.arcadeDrive(-mecanumDriveSubsystem.getAnglePID(CameraSubsystem.getInstance().vuforiaGetAngleRedTowerGoal()) * 1.5,
//                    mecanumDriveSubsystem.getDistancePID(mecanumDriveSubsystem.getAverageDistance()));
//            isReady = shooterSubsystem.atSetpoint() && mecanumDriveSubsystem.isAnglePIDAtSetpoint();
//        }
//        else
//        {
//            if(isReset && shooterSubsystem.getIR() && shooterSubsystem.atSetpoint()){
//                shooterSubsystem.loadToFlywheel();
//                shootTime = timer.seconds();
//                isReset = false;
//            }
//            else if (!isReset && (timer.seconds() - shootTime) > 0.75)
//            {
//                shooterSubsystem.resetServo();
//                isReset = true;
//            }
//        }
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        mecanumDriveSubsystem.stop();
//    }
//
//    @Override
//    public boolean isFinished() {
//        return false;
//    }
//}
