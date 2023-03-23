

package org.firstinspires.ftc.teamcode.drive;
//TODO FIX RIGHT SIDE
import android.app.Activity;
import android.view.View;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

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
public class ATONRIGHTSIDEWAYS extends LinearOpMode {
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
    int LEFT = 6;
    int MIDDLE = 4;
    int RIGHT = 2;
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
                camera.startStreaming(864, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        Arm arm = new Arm(hardwareMap, "turret", "wrist");
        org.firstinspires.ftc.teamcode.drive.Aton.LimitSwitch coneDectc = new org.firstinspires.ftc.teamcode.drive.Aton.LimitSwitch(hardwareMap, "limitSwitchA", false);
        PantherArm pantherArm = new PantherArm(arm,new org.firstinspires.ftc.teamcode.drive.Aton.LimitSwitch(hardwareMap,"limitSwitch",false),
                coneDectc,new org.firstinspires.ftc.teamcode.drive.Aton.LimitSwitch(hardwareMap,"armConeDet",true));


        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        AtomicInteger val = new AtomicInteger();
        wheelLat = hardwareMap.get(Servo.class, "deadwheelServo");
        wheelVer = hardwareMap.get(Servo.class, "deadwheelServo2");
        //wheelLat.setPosition(.9);
        //wheelVer.setPosition(.1);
        wheelLat.setPosition(.7);
        wheelVer.setPosition(.3);
        TrajectoryVelocityConstraint velConstraint1 = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(20)
        ));
        TrajectoryVelocityConstraint velConstraint2 = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(15)
        ));
        TrajectoryVelocityConstraint velConstraint3 = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(25)
        ));
        //Heads to cone and then reads the cone
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(velConstraint3)
                .forward(18)
                .strafeRight(43)
                .addTemporalMarker(pathTime -> pathTime * 0.75, () -> {
                    // Runs 50% into the path
                    arm.moveToTick(2800);
                })
                .build();
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(trajSeq.end())
                //.setVelConstraint(velConstraint2)
                .setVelConstraint(velConstraint3)
                .forward(3)
                .addTemporalMarker(pathTime -> pathTime * .999, () -> {
                    // Runs 99% into the path
                    arm.moveToTickT(2700);
                })
                .waitSeconds(2)
                .build();
        //placing first cone
        TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(trajSeq3.end())
                .setVelConstraint(velConstraint3)
                .forward(-2)
                .strafeRight(17)
                .addTemporalMarker(pathTime -> pathTime * 0.4, () -> {
                    arm.moveToTick(500);
                })
                .build();
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
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:"+tagOfInterest.id);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:"+tagOfInterest.id);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:"+tagOfInterest.id);
                }

            }

            telemetry.update();
            sleep(20);
        }
        /////////////////////////THIS IS WHERE IT STARTS/////EVERYTHING BEFORE HERE IS INITIALIZATION/////////////
        waitForStart();
        //changed last
        arm.openGripper();
        arm.moveToTick(100);
        sleep(10);
        if (isStopRequested()) return;

        drive.followTrajectorySequence(trajSeq);
        //drive.followTrajectorySequence(trajSeq2);
        drive.followTrajectorySequence(trajSeq3);
        arm.closeGripper();
        sleep(275);
        drive.followTrajectorySequence(trajSeq4);

        if (tagOfInterest.id==RIGHT){
            //does nothing
        }
        if (tagOfInterest.id==MIDDLE){
            TrajectorySequence MIDDLETraj = drive.trajectorySequenceBuilder(trajSeq4.end())
                    .lineToLinearHeading(new Pose2d(55, -5, Math.toRadians(90)))
                    .build();
            drive.followTrajectorySequence(MIDDLETraj);
        }
        if(tagOfInterest.id==LEFT){
            TrajectorySequence LEFTTraj = drive.trajectorySequenceBuilder(trajSeq4.end())
                    .lineToLinearHeading(new Pose2d(56, -25, Math.toRadians(90)))
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
}
