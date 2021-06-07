package org.ma.ftc.lib.control;

import org.ma.ftc.lib.control.MecanumDrive.MovementType;

public class Path {
    static final PathBuilder pathBuilder = new PathBuilder();

    static Movement[] mainPath;

    static final Movement[] failSafePath = pathBuilder
                                                    .strafeAngle(1.5, 45)
                                                    .getPath();

    static final Movement[] testPath = {
        new Movement(MovementType.TANK, 6.2, 0, 30, 5, 1, 1.2, 0.3),
    };

    static final Movement[] testPath2 = {
            new Movement(MovementType.TANK, 6, 0, 30, 5, 1, 1.2, 0),
            new Movement(MovementType.STRAFE, 2.8, 0, 30, 5, 1, 0.8, 0),

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
