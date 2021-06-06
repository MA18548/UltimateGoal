package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.ma.ftc.lib.command.RobotMap;
import org.ma.ftc.lib.command.SubsystemBase;

class ExampleSubsystem extends SubsystemBase {

    private static ExampleSubsystem exampleSubsystem;

    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    private ExampleSubsystem() {
        registerSubsystem();

        leftMotor = RobotMap.getInstance().getMap().get(DcMotor.class, "left_motor");
        rightMotor = RobotMap.getInstance().getMap().get(DcMotor.class, "right_motor");
    }

    public void setMotor(double power) {
        leftMotor.setPower(power);
    }

    public static ExampleSubsystem getInstance() {
        if (exampleSubsystem == null) {
            exampleSubsystem = new ExampleSubsystem();
        }
        return exampleSubsystem;
    }
}