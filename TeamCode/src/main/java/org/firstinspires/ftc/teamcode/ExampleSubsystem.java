package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ExampleSubsystem extends Subsystem {

    private static ExampleSubsystem exampleSubsystem;
    private DcMotor exampleMotor;

    private ExampleSubsystem()
    {
        exampleMotor = ExternalHardwareMap.getInstance().exampleMotor;
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
