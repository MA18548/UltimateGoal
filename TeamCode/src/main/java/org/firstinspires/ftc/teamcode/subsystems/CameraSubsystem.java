                                                                                                                                                                                                                                                                package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.ma.ftc.lib.command.RobotMap;
import org.ma.ftc.lib.command.SubsystemBase;
import org.ma.ftc.lib.control.TensorflowNavigation;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class CameraSubsystem extends SubsystemBase {
    private static CameraSubsystem cameraSubsystem;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AVEegHP/////AAABmezCHkPu0EMygK8/TGZXqyFVef+UsbiGSF1W0/si8o8bGrGJgvwcQq7abYeNF9RZI7ABWYorB0dJzX+KJnDzEVnnj4nkxZUktJn0RxGxjgOfUR4JhfDbbZYiKa3vv+UzcBXwoPYujI55lWm9/5WMSEUNmQDXG5jaBfJn4WXsbFafT/nCp2iN1EaA1ztw/P6a5CmSdfAgAP9xDPC8bX5VDaORhW0rXtPiO5OJYB2DKQG7XHwdJZy7XTF9SxTuEMkgcIh7uCW0cxL5ke266Sc+AUEtQbToMNz0KzxDQ9NR1/Sl49XYHXn4JJdMBvMBTIwH3+Zu5t/logfaU262+l5fV2GBNkbD9vLqJk9ind4cqRZu";

    private VuforiaLocalizer vuforiaLocalizer;
    private TFObjectDetector TFObjectDetector;

    private WebcamName intakeWebcam, shooterWebcam;
    private SwitchableCamera switchableCamera;

    private List<VuforiaTrackable> allTrackables;

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    private CameraSubsystem()
    {
        intakeWebcam = RobotMap.getInstance().getMap().get(WebcamName.class, "intake_webcam");
        shooterWebcam = RobotMap.getInstance().getMap().get(WebcamName.class, "shooter_webcam");

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "shooter_webcam");
                //ClassFactory.getInstance().getCameraManager()
                                      //      .nameForSwitchableCamera(intakeWebcam, shooterWebcam);

        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters);
        //switchableCamera.setActiveCamera(shooterWebcam);

        int tfodMonitorViewId = RobotMap.getInstance().getMap().appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", RobotMap.getInstance().getMap().appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        TFObjectDetector = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforiaLocalizer);
        TFObjectDetector.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsUltimateGoal = vuforiaLocalizer.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

//         Create a transformation matrix describing where the phone is on the robot.
//
//         Info:  The coordinate frame for the robot looks the same as the field.
//         The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
//         Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
//
//         For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z direction),
//         with the wide (horizontal) axis of the camera aligned with the X axis, and
//         the Narrow (vertical) axis of the camera aligned with the Y axis
//
//         But, this example assumes that the camera is actually facing forward out the front of the robot.
//         So, the "default" camera position requires two rotations to get it oriented correctly.
//         1) First it must be rotated +90 degrees around the X axis to get it horizontal (it's now facing out the right side of the robot)
//         2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
//
//         Finally the camera can be translated to its actual mounting position on the robot.
//              In this example, it is centered (left to right), but 4" forward of the middle of the robot, and 8" above ground level.

        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

        targetsUltimateGoal.activate();
        TFObjectDetector.activate();

        // The TensorFlow software will scale the input images from the camera to a lower resolution.
        // This can result in lower detection accuracy at longer distances (> 55cm or 22").
        // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
        // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
        // should be set to the value of the images used to create the TensorFlow Object Detection model
        // (typically 16/9).
        TFObjectDetector.setZoom(1, 16.0/9.0);


        registerSubsystem();
    }

    public void switchToIntakeCam()
    {
        switchableCamera.setActiveCamera(intakeWebcam);
    }

    public void switchToShooterCam()
    {
        switchableCamera.setActiveCamera(shooterWebcam);
    }

    public OpenGLMatrix vuforiaGetTransformationMatrix(String targetName)
    {
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                if (trackable.getName() == targetName)
                {
                    return ((VuforiaTrackableDefaultListener) trackable.getListener()).getFtcCameraFromTarget();
                }
            }
        }
        return null;
    }

    public double vuforiaGetDistanceFromName(String targetName)
    {
        OpenGLMatrix trans = vuforiaGetTransformationMatrix(targetName);
        if (trans != null)
        {
            VectorF translation = trans.getTranslation();
            return Math.abs(0 -translation.get(2)) + Math.abs(0 - translation.get(0));
        }
        return 0;
    }

    public double vuforiaGetDistanceRedTowerGoal()
    {
        return vuforiaGetDistanceFromName("Red Tower Goal Target");
    }

    public double vuforiaGetDistanceBlueTowerGoal()
    {
        return vuforiaGetDistanceFromName("Blue Tower Goal Target");
    }

    public double vuforiaGetAngleFromName(String targetName)
    {
        OpenGLMatrix trans = vuforiaGetTransformationMatrix(targetName);
        if (trans != null)
        {
            VectorF translation = trans.getTranslation();
            return Math.toDegrees(Math.atan2(translation.get(0), translation.get(2)));
        }
        return 0;
    }

    public double vuforiaGetAngleRedTowerGoal()
    {
        return vuforiaGetAngleFromName("Red Tower Goal Target");
    }

    public double vuforiaGetAngleBlueTowerGoal()
    {
        return vuforiaGetAngleFromName("Blue Tower Goal Target");
    }

    public List<Recognition> getTFOBObjects()
    {
        return TFObjectDetector.getRecognitions();
    }

    public String getTFOBLabel()
    {
        List<Recognition> recognitions = getTFOBObjects();
        return recognitions != null ? recognitions.get(0).getLabel() : null;
    }

    public static CameraSubsystem getInstance()
    {
        if (cameraSubsystem == null)
        {
            cameraSubsystem = new CameraSubsystem();
        }
        return cameraSubsystem;
    }

    @Override
    public void periodic() {
        telemetry.addData("Red Tower Goal Angle: ", vuforiaGetAngleRedTowerGoal());
        telemetry.addData("Red Tower Goal Distance: ", vuforiaGetDistanceRedTowerGoal());

    }
}
