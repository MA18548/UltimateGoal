package org.ma.ftc.lib.command; // MA FTC 18548

import com.qualcomm.robotcore.hardware.Gamepad;

import org.ma.ftc.lib.BooleanSupplier;

/**
 * Handles input from standard Joysticks connected to the Driver Station.
 *
 * <p>This class handles standard input that comes from the Driver Station by wrapping a {@link Gamepad}
 * class. It contains {@link Trigger} variables for each input on the gamepad.
 */
public class MAGamepad {
    public final Gamepad gamepad;

    public Trigger A;
    public Trigger B;
    public Trigger X;
    public Trigger Y;

    public Trigger LEFT_BUMPER;
    public Trigger RIGHT_BUMPER;

    public Trigger DPAD_UP;
    public Trigger DPAD_DOWN;
    public Trigger DPAD_LEFT;
    public Trigger DPAD_RIGHT;

    public Trigger START;
    public Trigger BACK;

    public Trigger LEFT_STICK;
    public Trigger RIGHT_STICK;

    public double joystickDeadzone = 0.1;

    public MAGamepad(final Gamepad gamepad) {
        this.gamepad = gamepad;

        this.A = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gamepad.a;
            }
        });
        this.B = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gamepad.b;
            }
        });
        this.X = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gamepad.x;
            }
        });
        this.Y = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gamepad.y;
            }
        });

        this.LEFT_BUMPER = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gamepad.left_bumper;
            }
        });
        this.RIGHT_BUMPER = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gamepad.right_bumper;
            }
        });

        this.DPAD_UP = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gamepad.dpad_up;
            }
        });
        this.DPAD_DOWN = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gamepad.dpad_down;
            }
        });
        this.DPAD_LEFT = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gamepad.dpad_left;
            }
        });
        this.DPAD_RIGHT = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gamepad.dpad_right;
            }
        });

        this.START = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gamepad.start;
            }
        });
        this.BACK = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gamepad.back;
            }
        });

        this.LEFT_STICK = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gamepad.left_stick_button;
            }
        });
        this.RIGHT_STICK = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gamepad.right_stick_button;
            }
        });
    }

    /**
     * Read the left stick Y axis value
     * <p>
     * {@return} The left stick Y axis value
     */
    public double getLeftY() {
        return Math.abs(gamepad.left_stick_y) >= joystickDeadzone ? gamepad.left_stick_y : 0;
    }

    /**
     * Read the left stick X axis value
     * <p>
     * {@return} The left stick X axis value
     */
    public double getLeftX() {
        return Math.abs(gamepad.left_stick_x) >= joystickDeadzone ? gamepad.left_stick_x : 0;
    }

    /**
     * Read the right stick Y axis value
     * <p>
     * {@return} The right stick Y axis value
     */
    public double getRightY() {
        return Math.abs(gamepad.right_stick_y) >= joystickDeadzone ? gamepad.right_stick_y : 0;
    }

    /**
     * Read the right stick X axis value
     * <p>
     * {@return} The right stick X axis value
     */
    public double getRightX() {
        return Math.abs(gamepad.right_stick_x) >= joystickDeadzone ? gamepad.right_stick_x : 0;
    }

    /**
     * Read the left stick Y axis value
     * <p>
     * {@return} The left stick Y axis value
     */
    public double getLeftY(double multiplier) {
        return getLeftY() * multiplier;
    }

    /**
     * Read the left stick X axis value
     * <p>
     * {@return} The left stick X axis value
     */
    public double getLeftX(double multiplier) {
        return getLeftX() * multiplier;
    }

    /**
     * Read the right stick Y axis value
     * <p>
     * {@return} The right stick Y axis value
     */
    public double getRightY(double multiplier) {
        return getRightY() * multiplier;
    }

    /**
     * Read the right stick X axis value
     * <p>
     * {@return} The right stick X axis value
     */
    public double getRightX(double multiplier) {
        return getRightX() * multiplier;
    }
}
