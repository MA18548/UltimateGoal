package org.ma.ftc.lib.command; // MA FTC 18548

import com.qualcomm.robotcore.hardware.Gamepad;

import org.ma.ftc.lib.BooleanSupplier;


public class Gamepad_EX
{
    private final Gamepad gamepad;

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

    public Gamepad_EX(Gamepad gamepad)
    {
        this.gamepad = gamepad;

        this.A = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return getAButtonAsBool();
            }
        });
        this.B = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return getBButtonAsBool();
            }
        });
        this.X = new Trigger(new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return getXButtonAsBool();
            }
        });
        this.Y = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return getYButtonAsBool();
            }
        });
    
        this.LEFT_BUMPER = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return getLeftBumperAsBool();
            }
        });
        this.RIGHT_BUMPER = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return getRightBumperAsBool();
            }
        });
    
        this.DPAD_UP = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return getUpDpadAsBool();
            }
        });
        this.DPAD_DOWN = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return getDownDpadAsBool();
            }
        });
        this.DPAD_LEFT = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return getLeftDpadAsBool();
            }
        });
        this.DPAD_RIGHT = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return getRightDpadAsBool();
            }
        });
    
        this.START = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return getStartButtonAsBool();
            }
        });
        this.BACK = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return getBackButtonAsBool();
            }
        });
    
        this.LEFT_STICK = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return getLeftStickButtonAsBool();
            }
        });
        this.RIGHT_STICK = new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return getRightStickButtonAsBool();
            }
        });
    }

    public boolean getAButtonAsBool()
    {
        return this.gamepad.a;
    }

    public boolean getBButtonAsBool()
    {
        return this.gamepad.b;
    }

    public boolean getXButtonAsBool()
    {
        return this.gamepad.x;
    }

    public boolean getYButtonAsBool()
    {
        return this.gamepad.y;
    }

    public boolean getLeftBumperAsBool()
    {
        return this.gamepad.left_bumper;
    }

    public boolean getRightBumperAsBool()
    {
        return this.gamepad.left_bumper;
    }

    public boolean getUpDpadAsBool()
    {
        return this.gamepad.dpad_up;
    }

    public boolean getDownDpadAsBool()
    {
        return this.gamepad.dpad_down;
    }

    public boolean getRightDpadAsBool()
    {
        return this.gamepad.dpad_right;
    }

    public boolean getLeftDpadAsBool()
    {
        return this.gamepad.dpad_left;
    }

    public boolean getStartButtonAsBool()
    {
        return this.gamepad.start;
    }

    public boolean getBackButtonAsBool()
    {
        return this.gamepad.back;
    }

    public boolean getLeftStickButtonAsBool()
    {
        return this.gamepad.left_stick_button;
    }

    public boolean getRightStickButtonAsBool()
    {
        return this.gamepad.right_stick_button;
    }

    public double getLeftY()
    {
        return gamepad.left_stick_y;
    }

    public double getLeftX()
    {
        return gamepad.left_stick_x;
    }

    public double getRightY()
    {
        return gamepad.right_stick_y;
    }

    public double getRightX()
    {
        return gamepad.right_stick_x;
    }


}
