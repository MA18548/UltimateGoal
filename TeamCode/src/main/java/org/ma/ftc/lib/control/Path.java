package org.ma.ftc.lib.control;

import org.ma.ftc.lib.control.MecanumDrive.MovementType;

public class Path {
    static final PathBuilder pathBuilder = new PathBuilder();
    public static Movement[] mainPath;

    static final Movement[] failSafePath = pathBuilder
                                                    .strafeAngle(1.5, 45)
                                                    .getPath();

    public static final Movement[] moveStraight = {
        new Movement(MovementType.TANK, 4.75, 0, 30, 5, 1, 1.2, 0),
    };

    public static final Movement[] strafeRight = {
            new Movement(MovementType.TANK, 4.7, 0, 30, 5, 1, 1.2, 0),
            new Movement(MovementType.STRAFE, 1.3, 0, 30, 5, 0.8, 1, 0),
    };

    public static final Movement[] strafeLeft = {
            new Movement(MovementType.TANK, 4.7, 0, 30, 5, 1, 1.2, 0),
            new Movement(MovementType.STRAFE, 1.3, 180, 30, 5, 0.8, 1, 0),
    };


//    static Movement[] testPath2 = {
//            new Movement(MovementType.STRAFE,3000, 45, 30, 2,
//                    1, 1),
//            new Movement(MovementType.TANK, 2000, 0, 60, 5,
//                    1, 1),
//            new Movement(MovementType.STRAFE, 2000, 180, 60, 5,
//                    1, 1),
//    };
    };
