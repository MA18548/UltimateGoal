package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.ma.ftc.lib.command.RobotMap;
import org.ma.ftc.lib.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {

    private static ExampleSubsystem exampleSubsystem;
    private final DcMotor exampleMotor;

    private ExampleSubsystem()
    {
        registerSubsystem();

        exampleMotor = RobotMap.getInstance().getMap().get(DcMotor.class, "motor");
    }

    public void setMotor(double power)
    {
        exampleMotor.setPower(power);
    }

    public static ExampleSubsystem getInstance()
    {
        if (exampleSubsystem == null)
        {
            exampleSubsystem = new ExampleSubsystem();
        }
        return exampleSubsystem;
    }
}