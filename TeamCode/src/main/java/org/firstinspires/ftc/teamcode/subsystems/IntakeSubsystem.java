package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.ma.ftc.lib.command.RobotMap;
import org.ma.ftc.lib.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private static IntakeSubsystem exampleSubsystem;

    private final DcMotor MainMotor;
//    private final VoltageSensor voltageSensor;

    private IntakeSubsystem() {
        registerSubsystem();

        MainMotor = RobotMap.getInstance().getMap().get(DcMotor.class, "intake_motor");
//        voltageSensor = (VoltageSensor) MainMotor;
    }

    public void setMotor(double power) {
        MainMotor.setPower(power);
    }

//    public double getVoltage()
//    {
//        return voltageSensor.getVoltage();
//    }

    public void pump(){

    }

    public  void release(){
        
    }

    public static IntakeSubsystem getInstance() {
        if (exampleSubsystem == null) {
            exampleSubsystem = new IntakeSubsystem();
        }
        return exampleSubsystem;
    }

    @Override
    public void periodic() {
//        telemetry.addData("Intake Voltage: ", getVoltage());
    }
}