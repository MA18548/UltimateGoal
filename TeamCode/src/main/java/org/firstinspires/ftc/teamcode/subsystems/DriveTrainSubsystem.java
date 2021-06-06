package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.ma.ftc.lib.command.SubsystemBase;

public class        DriveTrainSubsystem extends SubsystemBase {

    private static DriveTrainSubsystem exampleSubsystem;
    private DcMotor rightMotor;
    private DcMotor leftMotor;

    private DriveTrainSubsystem() {
        registerSubsystem();

        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
    }

    public void tankDrive(double left_power, double right_power) {
        rightMotor.setPower(left_power);
        leftMotor.setPower(right_power);
    }

    public void stop() {
        tankDrive(0, 0);
    }

    public void arcadeDrive(double angle, double distance) {
        double w = (100 - Math.abs(angle * 100)) * (distance) + distance * 100;
        double v = (100 - Math.abs(distance * 100)) * (angle) + angle * 100;

        tankDrive((-(v + w) / 200), ((v - w) / 200));
    }

    public static DriveTrainSubsystem getInstance() {
        if (exampleSubsystem == null) {
            exampleSubsystem = new DriveTrainSubsystem();
        }
        return exampleSubsystem;
    }
}