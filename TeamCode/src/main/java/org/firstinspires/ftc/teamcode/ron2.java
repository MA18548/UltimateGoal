package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class ron2 extends OpMode
{
    // Declare OpMode members.
    private DcMotor rightDrive;
    private DigitalChannel limitswitch;
    private ColorSensor colorSensor;
    private Servo servo;
    double rightPower;
    double tiks, previousTiks, counter;
    double servoState;
    boolean firstLimitSwitch;
    final int COLOR = 0;
    boolean reverse = false;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        servo = hardwareMap.get(Servo.class, "servo_motor");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        limitswitch = hardwareMap.digitalChannel.get("limit_switch");
        limitswitch.setMode(DigitalChannel.Mode.INPUT);
        servo.setPosition(0);
        servo.scaleRange(0, 1);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the batter
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPower = 0;
        tiks = 2500;
        previousTiks = 0;
        servoState = 0;
        counter = 1;
        firstLimitSwitch = limitswitch.getState();
        rightPower = 0.2;
        servoState = 0.2;

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (colorSensor.blue() != COLOR) {
            if (limitswitch.getState() || limitswitch.getState() != firstLimitSwitch) {
                if (gamepad2.a && !reverse){
                    rightPower = Range.clip(rightPower  * -1 , -1.0, 1.0);
                    reverse = true;
                }
                if (rightDrive.getCurrentPosition() >= previousTiks + tiks) {
                        counter++;
                        rightPower = Range.clip(rightPower + 0.2 * counter, -1.0, 1.0);
                        previousTiks = rightDrive.getCurrentPosition();
                        servoState = servoState + 0.2;
                    }
                    firstLimitSwitch = limitswitch.getState();
                }
                servo.setPosition(1 * servoState);
                rightDrive.setPower(rightPower);
                if (gamepad2.a && reverse){
                    reverse = true;
                }
                else if (!gamepad2.a && reverse){
                    reverse = false;
                }
        }
        else {
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            servo.setPosition(0);
            rightDrive.setPower(0);
        }

        telemetry.addData("Motor", "%f", rightPower);
        telemetry.addData("tiks", "%i",rightDrive.getCurrentPosition());
        telemetry.addData("servo position", "%f",servo.getPosition());
        telemetry.addData("color sensor" ,"%i",colorSensor.argb());
        telemetry.addData("limit switch state", "%b",limitswitch.getState());
        telemetry.update();
    }

    @Override
    public void stop() {
    }

}


