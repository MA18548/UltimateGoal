package org.ma.ftc.lib.command;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Singleton class that contains reference to general robot objects.
 *
 * <p>This singleton class contains references to the hardware map, runtime, gamepad(s) and telemetry
 * objects. It is initialized in the {@link RobotMap#init(HardwareMap, Telemetry, ElapsedTime, MAGamepad)}
 * function.
 */
public class RobotMap {
    private static RobotMap robotMap;

    private ElapsedTime runtime;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private MAGamepad drivingGamepad;
    private MAGamepad systemGamepad;

    private RobotMap() {
    }

    /**
     * Initializes the RobotMap with the general robot object references.
     *
     * @param hardwareMap Reference to a {@link com.qualcomm.robotcore.eventloop.opmode.OpMode}'s
     *                    hardware map object.
     * @param telemetry   Reference to a {@link com.qualcomm.robotcore.eventloop.opmode.OpMode}'s
     *                    telemetry object
     * @param runtime     Reference to a runtime object.
     * @param gamepad     Reference to a gamepad object.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime runtime,
                     MAGamepad drivingGamepad, MAGamepad systemGamepad) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.runtime = runtime;
        this.drivingGamepad = drivingGamepad;
        this.systemGamepad = systemGamepad;
    }

    /**
     * Gets the reference to the {@link HardwareMap} object.
     *
     * @return The reference to the hardware map.
     */
    public HardwareMap getMap() {
        return hardwareMap;
    }

    /**
     * Gets the reference to the {@link Telemetry} object.
     *
     * @return The reference to the telemetry.
     */
    public Telemetry getTelemtry() {
        return telemetry;
    }

    /**
     * Gets the reference to the {@link ElapsedTime} object.
     *
     * @return The reference to the elapsed time.
     */
    public ElapsedTime getRuntime() {
        return runtime;
    }

    /**
     * Gets the reference to the {@link MAGamepad} object.
     *
     * @return The reference to the gamepad.
     */
    public MAGamepad getDrivingGamepad() {
        return drivingGamepad;
    }

    public MAGamepad getSystemGamepad() {return systemGamepad;}

    /**
     * Returns the RobotMap instance.
     *
     * @return the instance
     */
    public static RobotMap getInstance() {
        if (robotMap == null) {
            robotMap = new RobotMap();
        }
        return robotMap;
    }
}
