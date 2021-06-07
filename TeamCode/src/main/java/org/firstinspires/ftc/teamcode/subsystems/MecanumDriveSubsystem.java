package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.ma.ftc.lib.MathUtil;
import org.ma.ftc.lib.command.SubsystemBase;
import org.ma.ftc.lib.control.MecanumDrive;
import org.ma.ftc.lib.control.MecanumDrive.MovementType;
import org.ma.ftc.lib.control.Movement;
import org.ma.ftc.lib.control.PIDController;
import org.ma.ftc.lib.geometry.Vector2D;

public class MecanumDriveSubsystem extends SubsystemBase {

    private static MecanumDriveSubsystem mecanumDriveSubsystem;

    private Movement currentMovement;

//    private final PIDFCoefficients leftFrontPIDCoefficents = new PIDFCoefficients(1, 1, 1, 1);
//    private final PIDFCoefficients leftBackPIDCoefficents = new PIDFCoefficients(1, 1, 1, 1);;
//    private final PIDFCoefficients rightFrontPIDCoefficents = new PIDFCoefficients(1, 1, 1, 1);;
//    private final PIDFCoefficients rightBackPIDCoefficents = new PIDFCoefficients(1, 1, 1, 1);;

    private final PIDController distancePID;
    private final PIDController anglePID;
    private final PIDController robotAnglePID;

    private final NavxMicroNavigationSensor navxMicro;
    private final IntegratingGyroscope gyro;

    public final DcMotorEx leftFrontMotor;
    private final DcMotorEx leftBackMotor;
    private final DcMotorEx rightFrontMotor;
    private final DcMotorEx rightBackMotor;

    private final MecanumDrive mecanumDrive;

    public final double TICKS_PER_METER = 750;


    /**
     * Creates a new ExampleSubsystem.
     */
    private MecanumDriveSubsystem() {
        registerSubsystem();

        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "left_back");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "right_front");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "right_back");

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




//
//        leftFrontMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, leftFrontPIDCoefficents);
//        leftBackMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, leftBackPIDCoefficents);
//        rightFrontMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, rightFrontPIDCoefficents);
//        rightBackMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, rightBackPIDCoefficents);

        mecanumDrive = new MecanumDrive(leftFrontMotor, leftBackMotor,
                                        rightFrontMotor, rightBackMotor);

        anglePID = new PIDController(0, 3, 3.1e-2, 0, 3e-5, 0.001);
        robotAnglePID = new PIDController(0, 3, 1.8e-2, 0, 5e-3, 0);
        distancePID = new PIDController(0, 60, 1e-2, 0, 2e-5, 0.001);

        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = navxMicro;
    }

    public void setMotors(double leftFront, double leftBack, double rightFront, double rightBack) {
        mecanumDrive.setMotors(leftFront, leftBack, rightFront, rightBack);
    }

    public void reset()
    {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        anglePID.setSetpoint(0);
        distancePID.setSetpoint(0);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void stop() {
        mecanumDrive.stop();
    }

    public void tankDrive(double leftPower, double rightPower) {
        mecanumDrive.setMotors(leftPower, leftPower,
                                rightPower, rightPower);
    }

    public void arcadeDrive(double angle, double distance) {
        mecanumDrive.arcadeDrive(angle, distance);
    }

    public void driveCartesian(double ySpeed, double xSpeed, double rotation) {
        mecanumDrive.driveCartesian(ySpeed, xSpeed, rotation);
    }

    public void drivePolar(double mag, double angle, double rotation) {
        mecanumDrive.drivePolar(mag, angle, rotation);
    }

    public void fixedDrivePolar(double mag, double angle)
    {
        double ySpeed = mag * Math.sin(angle);
        double xSpeed = mag * Math.cos(angle);

        ySpeed = MathUtil.clamp(ySpeed, 1f, -1f);
        xSpeed = MathUtil.clamp(xSpeed, 1f, -1f);

        Vector2D input = new Vector2D(xSpeed, ySpeed);

        double[] wheelSpeeds = new double[4];
        double leftFrontRotation = !currentMovement.isMainCrossMotors() ? -robotAnglePID.getPID(getAngle()) : 0;
        double leftBackRotation =  !currentMovement.isSecondaryCrossMotors() ? -robotAnglePID.getPID(getAngle()) : 0;
        double rightFrontRotation =  !currentMovement.isSecondaryCrossMotors() ? robotAnglePID.getPID(getAngle()) : 0;
        double rightBackRotation =  !currentMovement.isMainCrossMotors() ? robotAnglePID.getPID(getAngle()) : 0;

        wheelSpeeds[0] = input.getX() + input.getY() + leftFrontRotation * currentMovement.getAngleSpeed();
        wheelSpeeds[2] = -input.getX() + input.getY() + rightFrontRotation * currentMovement.getAngleSpeed();
        wheelSpeeds[1] = -input.getX() + input.getY() + leftBackRotation * currentMovement.getAngleSpeed();
        wheelSpeeds[3] = input.getX() + input.getY() + rightBackRotation * currentMovement.getAngleSpeed();
        MathUtil.normalize(wheelSpeeds);

        setMotors(wheelSpeeds[0], wheelSpeeds[1],
                wheelSpeeds[2], wheelSpeeds[3]);
    }

    public void powerCalc(double y, double x, double rx) {
        double leftFrontPower = y + x + rx;
        double leftBackPower = y - x + rx;
        double rightFrontPower = y - x - rx;
        double rightBackPower = y + x - rx;

        // Put powers in the range of -1 to 1 only if they aren't already (not
        // checking would cause us to always drive at full speed)

        if (Math.abs(leftFrontPower) > 1 || Math.abs(leftBackPower) > 1 ||
                Math.abs(rightFrontPower) > 1 || Math.abs(rightBackPower) > 1) {
            double max = 0;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower));
            max = Math.max(Math.abs(rightFrontPower), max);
            max = Math.max(Math.abs(rightBackPower), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        setMotors(leftFrontPower, leftBackPower,
                rightFrontPower, rightBackPower);
    }

    public double getTicksPerMinute() {
        return 1;
    }

    public double getAverageDistance() {
        return (leftFrontMotor.getCurrentPosition() + leftBackMotor.getCurrentPosition()+ rightFrontMotor.getCurrentPosition() + rightBackMotor.getCurrentPosition()) / 4.0;
    }

    public double getAngle() {
        return normalizeDegrees(
                gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    }

    public void setAngleSetpoint(double setpoint)
    {
        anglePID.setSetpoint(setpoint);
    }

    public void setAngleTolerance(double tolerance)
    {
        anglePID.setTolerance(tolerance);
    }

    public void setDistanceSetpoint(double setpoint)
    {
        distancePID.setSetpoint(setpoint);
    }

    public double getDistancePID() {
        return distancePID.getPID(getDistance());
    }

    public double getDistancePID(double input)
    {
        return distancePID.getPID(input);
    }

    public double getAnglePID() {
        return anglePID.getPID(getAngle());
    }

    public double getAnglePID(double input) {
        return anglePID.getPID(input);
    }

    public double getDistanceError() {
        return distancePID.getError();
        }

    public boolean isDistanceDone()
    {
        return Math.abs(getDistanceError()) <= currentMovement.getDistanceTolerance();
    }

    public boolean isAngleSetpoint()
    {
        return Math.abs(getAngleError()) <= anglePID.getTolerance();
    }

    public boolean isAngleDone()
    {
        return currentMovement.getMovementType() != MovementType.TANK ||  Math.abs(getAngleError()) <= currentMovement.getAngleTolerance();
    }

    public double getDistance()
    {
        if (currentMovement.getMovementType() == MovementType.TANK)
        {
            return getAverageDistance();
        }

        double encoder;
        double targetAngle = currentMovement.getAngleSetpoint();
        if (currentMovement.isMainCrossMotors())
        {
            encoder = getMainCrossAverage();
        }
        else
        {
            encoder = getSecondaryCrossAverage();
        }
        return encoder;
    }

    public double getMainCrossAverage()
    {
        return (leftFrontMotor.getCurrentPosition() + rightBackMotor.getCurrentPosition())/2.0;
    }

    public double getSecondaryCrossAverage()
    {
        return (rightFrontMotor.getCurrentPosition() + leftBackMotor.getCurrentPosition())/2.0;
    }
    public double getAngleError() {
        return anglePID.getError();
    }

    public boolean IsGyroCalibrating() {
        return navxMicro.isCalibrating();
    }

    public boolean isAnglePIDAtSetpoint() {return anglePID.atSetpoint();}

    public void setMovementBySetpoint(Movement movement) {
        this.currentMovement = movement;
        distancePID.reset();
        anglePID.reset();
        robotAnglePID.reset();
        distancePID.setSetpoint(getDistance() + currentMovement.getDistanceSetpoint() * currentMovement.getDirection());
        distancePID.setTolerance(currentMovement.getDistanceTolerance());
        if (currentMovement.isStrafe())
        {
            robotAnglePID.setSetpoint(getAngle());
            robotAnglePID.setTolerance(currentMovement.getAngleTolerance());
        }
        else
        {
            anglePID.setSetpoint(currentMovement.getAngleSetpoint());
            anglePID.setTolerance(currentMovement.getAngleTolerance());
        }

    }
    
    public Movement getCurrentMovement() {
        return currentMovement;
    }

    public void runMovement() {
        switch (currentMovement.getMovementType())
        {
            case TANK:
                double angle = getAnglePID() * currentMovement.getAngleSpeed();
                double distance = getDistancePID() * currentMovement.getDistanceSpeed();
                arcadeDrive(angle,
                        distance);
                break;
            case STRAFE:
                fixedDrivePolar(getDistancePID() * currentMovement.getDirection(), Math.toRadians(currentMovement.getAngleSetpoint()));
                break;
        }
    }

    public static double normalizeDegrees(double degrees)
    {
//        while (degrees >= 180.0) degrees -= 360.0;
//        while (degrees < -180.0) degrees += 360.0;
        return degrees;
    }


    public static MecanumDriveSubsystem getInstance() {
        if (mecanumDriveSubsystem == null) {
            mecanumDriveSubsystem = new MecanumDriveSubsystem();
        }
        return mecanumDriveSubsystem;
    }

    public void periodic() {
        // This method will be called once per scheduler
        telemetry.addData("distance: ", getAverageDistance());
//        telemetry.addData("angle:", getAngle());
////
//        telemetry.addData("distance setpoint: ", distancePID.getSetpoint());
//        telemetry.addData("angle pid: ", anglePID);
//
//        telemetry.addData("left front motor power", leftFrontMotor.getPower());
//        telemetry.addData("left back motor power", leftBackMotor.getPower());
//        telemetry.addData("right front motor power", rightFrontMotor.getPower());
//        telemetry.addData("right back motor power", rightBackMotor.getPower());
//
//        telemetry.addData("left front encoder", leftFrontMotor.getCurrentPosition());
//        telemetry.addData("left back encoder", leftBackMotor.getCurrentPosition());
//        telemetry.addData("right front encoder", rightFrontMotor.getCurrentPosition());
//        telemetry.addData("right back encoder", rightBackMotor.getCurrentPosition());
//
//        telemetry.addData("left front mode", leftFrontMotor.getMode());
//        telemetry.addData("left back mode", leftBackMotor.getMode());
//        telemetry.addData("right front mode", rightFrontMotor.getMode());
//        telemetry.addData("right back mode", rightBackMotor.getMode());
        //        RobotMap.getInstance().getTelemtry().addData("movement: ", currentMovement);
    }
}
