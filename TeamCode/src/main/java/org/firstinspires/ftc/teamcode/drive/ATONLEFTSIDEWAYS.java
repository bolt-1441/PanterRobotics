

package org.firstinspires.ftc.teamcode.drive;

        import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

        import android.app.Activity;
        import android.graphics.Color;
        import android.view.View;

        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.acmerobotics.roadrunner.geometry.Vector2d;
        import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
        import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
        import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DistanceSensor;
        import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
        import com.qualcomm.robotcore.hardware.NormalizedRGBA;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.SwitchableLight;

        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
        import org.openftc.apriltag.AprilTagDetection;
        import org.openftc.easyopencv.OpenCvCamera;
        import org.openftc.easyopencv.OpenCvCameraFactory;
        import org.openftc.easyopencv.OpenCvCameraRotation;

        import java.util.ArrayList;
        import java.util.Arrays;
        import java.util.concurrent.atomic.AtomicInteger;

@Autonomous(group = "drive")
public class ATONLEFTSIDEWAYS extends LinearOpMode {
    private Servo wrist = null;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 18 from the 36h11 family
    int LEFT = 2;
    int MIDDLE = 4;
    int RIGHT = 6;
    AprilTagDetection tagOfInterest = null;
    private DcMotor turret = null;
    NormalizedColorSensor colorSensor;
    View relativeLayout;

    public Servo wheelLat;
    public Servo wheelVer;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(864, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                }

            }

            telemetry.update();
            sleep(20);
        }
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);
        Arm arm = new Arm(hardwareMap, "turret", "wrist");
        org.firstinspires.ftc.teamcode.drive.Aton.LimitSwitch coneDectc = new org.firstinspires.ftc.teamcode.drive.Aton.LimitSwitch(hardwareMap, "limitSwitchA", false);
        PantherArm pantherArm = new PantherArm(arm,new org.firstinspires.ftc.teamcode.drive.Aton.LimitSwitch(hardwareMap,"limitSwitch",false),
                coneDectc,new org.firstinspires.ftc.teamcode.drive.Aton.LimitSwitch(hardwareMap,"armConeDet",true));

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        wrist = hardwareMap.get(Servo.class, "wrist");
        turret = hardwareMap.get(DcMotor.class, "turret");
        LineTracker lineTrackerLeftForward = new LineTracker(hardwareMap, "lineTrackerLeftForward");
        LineTracker lineTrackerRightForward = new LineTracker(hardwareMap, "lineTrackerRightForward");
        LineTracker lineTrackerLeftBack = new LineTracker(hardwareMap, "lineTrackerRightBack");
        LineTracker lineTrackerRightBack = new LineTracker(hardwareMap, "lineTrackerLeftBack");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        runSample(); // actually execute the sample
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        AtomicInteger val = new AtomicInteger();
        wheelLat = hardwareMap.get(Servo.class, "deadwheelServo");
        wheelVer = hardwareMap.get(Servo.class, "deadwheelServo2");
        //wheelLat.setPosition(.9);
        //wheelVer.setPosition(.1);
        wheelLat.setPosition(.8);
        wheelVer.setPosition(.2);
        /////////////////////////THIS IS WHERE IT STARTS/////EVERYTHING BEFORE HERE IS INITIALIZATION/////////////
        waitForStart();
        //changed last
        arm.openGripper();
        arm.moveToTick(100);
        sleep(10);
        if (isStopRequested()) return;
        TrajectoryVelocityConstraint velConstraint1 = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(20)
        ));
        TrajectoryVelocityConstraint velConstraint2 = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(10)
        ));
        TrajectoryVelocityConstraint velConstraint3 = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(3)
        ));
        //Heads to cone and then reads the cone
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(velConstraint1)
                .forward(-3)
                .build();
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq.end())
                .strafeLeft(43)
                .addTemporalMarker(pathTime -> pathTime * 0.5, () -> {
                    // Runs 50% into the path
                    arm.moveToTick(2100);
                })
                .build();
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(trajSeq2.end())
                .setVelConstraint(velConstraint2)
                .forward(3)
                .addTemporalMarker(pathTime -> pathTime * .999, () -> {
                    // Runs 99% into the path
                    arm.moveToTickT(1800);
                })
                .build();
        TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(trajSeq3.end())
                .strafeLeft(11)
                .addTemporalMarker(pathTime -> pathTime * 0.7, () -> {
                    arm.moveToTick(500);
                })
                .build();
        TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(trajSeq4.end())
//                .setVelConstraint(velConstraint2)
//                .setTurnConstraint(3,1)
//                .lineToLinearHeading(new Pose2d(55, 5, Math.toRadians(0)))
//                .waitSeconds(.4)
//                .setVelConstraint(velConstraint3)
                .turn(Math.toRadians(180),5,2.5)
                .lineToLinearHeading(new Pose2d(58, 18, Math.toRadians(90)))
                .build();
        TrajectorySequence trajComp = drive.trajectorySequenceBuilder(trajSeq5.end())
                .lineToLinearHeading(new Pose2d(58+val.get(), 19, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(58+val.get(), 25, Math.toRadians(90)))
                .setVelConstraint(velConstraint3)
                .forward(4)
                .forward(-1)
                .build();
        // This example will run 50% of the way through the path
        TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(trajComp.end())
                .lineToLinearHeading(new Pose2d(58+val.get(), -17, Math.toRadians(90)))
                .strafeRight(14)
                .build();
        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(trajSeq6.end())
                .setVelConstraint(velConstraint3)
                .forward(3)
                .build();
        TrajectorySequence trajSeq8 = drive.trajectorySequenceBuilder(trajSeq7.end())
                .forward(-5)
                .build();
        TrajectorySequence trajSeq9 = drive.trajectorySequenceBuilder(trajSeq8.end())
                .strafeLeft(14)
                        .build();

        drive.followTrajectorySequence(trajSeq);
        sleep(10);
        drive.followTrajectorySequence(trajSeq2);
        sleep(10);
        drive.followTrajectorySequence(trajSeq3);
        arm.closeGripper();
        sleep(200);
        arm.moveToTick(1900);
        drive.followTrajectorySequence(trajSeq4);
        sleep(50);
        drive.followTrajectorySequence(trajSeq5);
        if(lineTrackerLeftForward.isOnLine()||lineTrackerRightForward.isOnLine())
            val.addAndGet(-3);
        if (lineTrackerRightBack.isOnLine()||lineTrackerLeftBack.isOnLine())
            val.addAndGet(3);
        else
            val.set(0);
        telemetry.addData("val",val.get());
        telemetry.update();
        arm.moveToTickT(1200);
        drive.followTrajectorySequence(trajComp);
        sleep(50);
        pantherArm.grabCone();
        sleep(100);
        drive.followTrajectorySequence(trajSeq6);
        sleep(50);
        arm.moveToTick(2900);
        drive.followTrajectorySequence(trajSeq7);
        arm.moveToTick(2800);
        arm.closeGripper();
        arm.moveToTick(2900);
        drive.followTrajectorySequence(trajSeq8);
        arm.moveToTick(300);
        drive.followTrajectorySequence(trajSeq9);
        if (tagOfInterest.id==RIGHT){
        }
        if (tagOfInterest.id==MIDDLE){
            TrajectorySequence MIDDLETraj = drive.trajectorySequenceBuilder(trajSeq8.end())
                    .lineToLinearHeading(new Pose2d(58, 6, Math.toRadians(90)))
                    .build();
            drive.followTrajectorySequence(MIDDLETraj);
        }
        if (tagOfInterest.id==LEFT){
            TrajectorySequence LEFTTraj = drive.trajectorySequenceBuilder(trajSeq8.end())
                    .lineToLinearHeading(new Pose2d(58, 26, Math.toRadians(90)))
                    .build();
            drive.followTrajectorySequence(LEFTTraj);
        }

        sleep(5000);

    }
    //HOW TO MAKE THE ROBOT MOVE TO A SPECIFIC SPOT//
//    drive.followTrajectory(
//            drive.trajectoryBuilder(traj.end(), true)
//            .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//            .build()
//        );
    protected int runSample() {
        // You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
        // normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
        // can give very low values (depending on the lighting conditions), which only use a small part
        // of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
        // you should use a smaller gain than in dark conditions. If your gain is too high, all of the
        // colors will report at or near 1, and you won't be able to determine what color you are
        // actually looking at. For this reason, it's better to err on the side of a lower gain
        // (but always greater than  or equal to 1).
        float gain = 5;
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        // for an explanation of HSV color.
        final float[] hsvValues = new float[3];

        // xButtonPreviouslyPressed and xButtonCurrentlyPressed keep track of the previous and current
        // state of the X button on the gamepad
        boolean xButtonPreviouslyPressed = false;
        boolean xButtonCurrentlyPressed = false;

        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        // Wait for the start button to be pressed.
        waitForStart();

        // Loop until we are asked to stop
        // while (opModeIsActive()) {
        // Explain basic gain information via telemetry
        //telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
        telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");

        // Show the gain value via telemetry
        telemetry.addData("Gain", gain);

        // Tell the sensor our desired gain value (normally you would do this during initialization,
        // not during the loop)
        colorSensor.setGain(gain);

        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
         * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
         * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
         * for an explanation of HSV color. */

        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues);

        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors.alpha);

        /* If this color sensor also has a distance sensor, display the measured distance.
         * Note that the reported distance is only useful at very close range, and is impacted by
         * ambient light and surface reflectivity. */
        if (colorSensor instanceof DistanceSensor) {
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(CM));
        }

        telemetry.update();

        // Change the Robot Controller's background color to match the color detected by the color sensor.
        relativeLayout.post(() -> relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues)));
        double red = colors.red;
        double green = colors.green;
        double blue = colors.blue;
        if(red>green)
            if(red>blue)
                return 1;
        if(green>red)
            if(green>blue)
                return 2;
        if(blue>red)
            if(blue>green)
                return 3;
        return 4;
        //}
    }
}
