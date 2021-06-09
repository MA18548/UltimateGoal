package org.ma.ftc.lib.control; /* MA FTC 18548
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.ma.ftc.lib.MathUtil;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "18548 - Arcade Drive", group = "Iterative Opmode")
@Disabled
public class ArcadeDrive extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime;

    private final PIDController pidControllerAngle = new PIDController(180, 2, .027, 0, .001);
    private final PIDController pidControllerDistance = new PIDController(4000, 50, .007, 0, .0001);

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private final DcMotor elevator = null;
    private final Servo servoLeft = null;
    private final Servo servoRight = null;

    private final DigitalChannel elevatorLeftHall = null;
    private final DigitalChannel elevatorRightHall = null;

    private NavxMicroNavigationSensor navxMicro;
    private int path = 0;
    IntegratingGyroscope gyro;

    private final boolean gateClosed = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightDrive = hardwareMap.get(DcMotor.class, "rightFront");

//        elevator = hardwareMap.get(DcMotor.class, "elevator");
//
//        servoLeft = hardwareMap.get(Servo.class, "servo_left");
//        servoRight = hardwareMap.get(Servo.class, "servo_right");
//
//
//        elevatorLeftHall = hardwareMap.digitalChannel.get("left_hall_effect");
//        elevatorRightHall = hardwareMap.digitalChannel.get("right_hall_effect");

        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = navxMicro;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


//        servoLeft.setDirection(Servo.Direction.FORWARD);
//        servoRight.setDirection(Servo.Direction.REVERSE);
//
//        elevatorLeftHall.setMode(DigitalChannel.Mode.INPUT);
//        elevatorRightHall.setMode(DigitalChannel.Mode.INPUT);


        this.runtime = new ElapsedTime();

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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;


        AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftDrive.getPower(), rightDrive.getPower());
        telemetry.addData("Encoders", "left (%.2f), right (%.2f)", (float) leftDrive.getCurrentPosition(), (float) rightDrive.getCurrentPosition());
        telemetry.addData("Encoder average", getAverageDrive());


        telemetry.addLine()
                .addData("dx", formatRate(rates.xRotationRate))
                .addData("dy", formatRate(rates.yRotationRate))
                .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));

        telemetry.addLine()
                .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle))
                .addData("roll", formatAngle(angles.angleUnit, angles.secondAngle))
                .addData("pitch", "%s deg", formatAngle(angles.angleUnit, angles.thirdAngle));


        telemetry.addData("Angle PID setpoint", pidControllerAngle.getSetpoint());
        telemetry.addData("Angle PID tolerance", pidControllerAngle.getTolerance());
        telemetry.addData("Angle PID at Setpoint", pidControllerAngle.atSetpoint());
        telemetry.addData("Angle PID error", pidControllerAngle.getError());

        telemetry.addData("Distance PID setpoint", pidControllerDistance.getSetpoint());
        telemetry.addData("Distance PID tolerance", pidControllerDistance.getTolerance());
        telemetry.addData("Distance PID at Setpoint", pidControllerDistance.atSetpoint());
        telemetry.addData("Distance PID error", pidControllerDistance.getError());

        telemetry.update();

        double anglePID = MathUtil.clamp(pidControllerAngle.getPID(angles.firstAngle), -.75, .75);
        double distancePID = 0; //pidControllerDistance.getPID(getAverageDrive());


        if (gamepad1.b) //&& !pidControllerDistance.atSetpoint())
        {
            switch (path) {
                case 0:
                    Auto(pidControllerAngle, pidControllerDistance, gyro, 180, 0);
                    if ((pidControllerAngle.atSetpoint() && pidControllerDistance.atSetpoint())) {
                        path++;
                    }
                    break;

                case 1:
                    Auto(pidControllerAngle, pidControllerDistance, gyro, 90, 0);
                    if ((pidControllerAngle.atSetpoint() && pidControllerDistance.atSetpoint())) {
                        path++;
                    }
                    break;
                case 2:
                    Auto(pidControllerAngle, pidControllerDistance, gyro, -90, 0);
                    if ((pidControllerAngle.atSetpoint() && pidControllerDistance.atSetpoint())) {
                        path++;
                    }
                    break;
                case 3:
                    Auto(pidControllerAngle, pidControllerDistance, gyro, 0, 1000);
                    if ((pidControllerAngle.atSetpoint() && pidControllerDistance.atSetpoint())) {
                        path++;
                    }
                    break;
                case 4:
                    Auto(pidControllerAngle, pidControllerDistance, gyro, 0, 0);
                    if ((pidControllerAngle.atSetpoint() && pidControllerDistance.atSetpoint())) {
                        path++;
                    }
                    break;
                default:
                    tankDrive(0d, 0d);
                    break;
            }
        } else if (gamepad1.a) {
            resetEncoders();

            // TODO reset gyro
        } else {
            leftDrive.setPower(-gamepad1.right_stick_y);
            rightDrive.setPower(-gamepad1.left_stick_y);
        }
//        elevator.setPower(elevatorPower);
//        if (elevatorLeftState == true) {
//            elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//        if (gamepad2.y) {
//            gateClosed = !gateClosed;
//
//            servoLeft.setPosition(gateClosed ? 1 : 0);
//            servoRight.setPosition(gateClosed ? 1 : 0);
//        }
//        telemetry.addData("Hall Effects:", "Left Hall Effect: %b, Right Hall Effect: %b", elevatorLeftState, elevatorRightState);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void tankDrive(double leftPower, double rightPower) {
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void arcadeDrive(double angle, double distance) {
        double w = (100 - Math.abs(angle * 100)) * (distance) + distance * 100;
        double v = (100 - Math.abs(distance * 100)) * (angle) + angle * 100;

        tankDrive((-(v + w) / 200), ((v - w) / 200));
    }

    public double getAverageDrive() {
        return (leftDrive.getCurrentPosition() + rightDrive.getCurrentPosition()) / 2.0;
    }

    public void resetEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void Auto(PIDController angle, PIDController distance, IntegratingGyroscope gyro, double angleSetpoint, double distanceSetpoint) {
        angle.setSetpoint(angleSetpoint);
        distance.setSetpoint(distanceSetpoint);

        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double anglePID = MathUtil.clamp(pidControllerAngle.getPID(angles.firstAngle), -.75, .75);
        double distancePID = distance.getPID(getAverageDrive());

        arcadeDrive(anglePID, distancePID);
    }

    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}