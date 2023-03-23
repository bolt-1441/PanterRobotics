

package org.firstinspires.ftc.teamcode.drive;

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
public class AutonomousLeftV2 extends LinearOpMode {
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
                new TranslationalVelocityConstraint(3)
        ));
        //Heads to cone and then reads the cone
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(-24)
                .strafeRight(40)
                .addTemporalMarker(pathTime -> pathTime * 0.75, () -> {
                    // Runs 50% into the path
                    arm.moveToTick(2100);
                })
                .build();
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(trajSeq.end())
                //.setVelConstraint(velConstraint2)
                .forward(2)
                .addTemporalMarker(pathTime -> pathTime * .999, () -> {
                    // Runs 99% into the path
                    arm.moveToTickT(1800);
                })
                .build();
        //placing first cone
        TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(trajSeq3.end())
                .forward(-2)
                .strafeRight(17)
                .addTemporalMarker(pathTime -> pathTime * 0.4, () -> {
                    arm.moveToTick(500);
                })
                .build();
        TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(trajSeq4.end())
//                .setVelConstraint(velConstraint2)
//                .setTurnConstraint(3,1)
//                .lineToLinearHeading(new Pose2d(55, 5, Math.toRadians(0)))
//                .waitSeconds(.4)
//                .setVelConstraint(velConstraint3)
                .lineToLinearHeading(new Pose2d(55, 12, Math.toRadians(90)))
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
        arm.moveToTick(2400);
        drive.followTrajectorySequence(trajSeq4);
        drive.followTrajectorySequence(trajSeq5);
        LineTracker lineTrackerLeftForward = new LineTracker(hardwareMap, "lineTrackerLeftForward");
        LineTracker lineTrackerRightForward = new LineTracker(hardwareMap, "lineTrackerRightForward");
        LineTracker lineTrackerLeftBack = new LineTracker(hardwareMap, "lineTrackerRightBack");
        LineTracker lineTrackerRightBack = new LineTracker(hardwareMap, "lineTrackerLeftBack");
        if(lineTrackerLeftForward.isOnLine()||lineTrackerRightForward.isOnLine())
            val.addAndGet(0);
        if (lineTrackerRightBack.isOnLine()||lineTrackerLeftBack.isOnLine())
            val.addAndGet(-1);
        else
            val.set(1);
        //getting cone from stack
        TrajectorySequence trajComp = drive.trajectorySequenceBuilder(trajSeq5.end())
                .lineToLinearHeading(new Pose2d(55+val.get(), 22, Math.toRadians(90)))
                .setVelConstraint(velConstraint2)
                .forward(2)
                .forward(-1)
                .build();
        // This example will run 50% of the way through the path
        TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(trajComp.end())
                .lineToLinearHeading(new Pose2d(55+val.get(), 2, Math.toRadians(90)))
                .strafeLeft(13)
                .build();
        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(trajSeq6.end())
                //.setVelConstraint(velConstraint3)
                .forward(1)
                .build();
        TrajectorySequence trajSeq8 = drive.trajectorySequenceBuilder(trajSeq7.end())
                .forward(-1)
                .build();
        TrajectorySequence trajSeq9 = drive.trajectorySequenceBuilder(trajSeq8.end())
                .strafeRight(12)
                .build();
        TrajectorySequence trajSeq10 = drive.trajectorySequenceBuilder(trajSeq9.end())
                .lineToLinearHeading(new Pose2d(54, 12, Math.toRadians(90)))
                .build();
        sleep(50);
        telemetry.addData("val",val.get());
        telemetry.update();
        arm.moveToTickT(1200);
        drive.followTrajectorySequence(trajComp);
        pantherArm.grabConeAton();
        sleep(50);
        drive.followTrajectorySequence(trajSeq6);
        sleep(50);
        arm.moveToTick(1300);
        drive.followTrajectorySequence(trajSeq7);
        arm.moveToTick(1300);
        arm.closeGripper();
        sleep(100);
        arm.moveToTick(1300);
        drive.followTrajectorySequence(trajSeq8);
        arm.moveToTick(1100);
        drive.followTrajectorySequence(trajSeq9);
        arm.moveToTick(300);
        drive.followTrajectorySequence(trajSeq10);
        sleep(50);
        //to stack part 2
        if(lineTrackerLeftForward.isOnLine()||lineTrackerRightForward.isOnLine())
            val.addAndGet(0);
        if (lineTrackerRightBack.isOnLine()||lineTrackerLeftBack.isOnLine())
            val.addAndGet(-1);
        else
            val.set(1);
        TrajectorySequence trajComp2 = drive.trajectorySequenceBuilder(trajSeq10.end())
                .lineToLinearHeading(new Pose2d(55+val.get(), 26
                        , Math.toRadians(90)))
                .setVelConstraint(velConstraint2)
                .forward(-1)
                .build();
        // This example will run 50% of the way through the path
        TrajectorySequence trajSeq11 = drive.trajectorySequenceBuilder(trajComp2.end())
                .lineToLinearHeading(new Pose2d(55+val.get(), 2, Math.toRadians(90)))
                .strafeLeft(14)
                .build();
        TrajectorySequence trajSeq12 = drive.trajectorySequenceBuilder(trajSeq11.end())
                //.setVelConstraint(velConstraint2)
                .forward(3)
                .build();
        TrajectorySequence trajSeq13 = drive.trajectorySequenceBuilder(trajSeq12.end())
                .forward(-3)
                .build();
        TrajectorySequence trajSeq14 = drive.trajectorySequenceBuilder(trajSeq13.end())
                .strafeRight(15)
                .build();

        telemetry.addData("val",val.get());
        telemetry.update();
        arm.moveToTickT(1200);
        drive.followTrajectorySequence(trajComp2);
        sleep(50);
        pantherArm.grabConeAton();
        sleep(100);
        drive.followTrajectorySequence(trajSeq11);
        sleep(50);
        arm.moveToTick(1300);
        drive.followTrajectorySequence(trajSeq12);
        arm.moveToTick(1300);
        arm.closeGripper();
        arm.moveToTick(1100);
        drive.followTrajectorySequence(trajSeq13);
        arm.moveToTick(1000);
        drive.followTrajectorySequence(trajSeq14);
        arm.moveToTick(300);
        if (tagOfInterest.id==RIGHT){
            //does nothing
            TrajectorySequence RIGHTraj = drive.trajectorySequenceBuilder(trajSeq8.end())
                    .lineToLinearHeading(new Pose2d(53, 25, Math.toRadians(90)))
                    .build();
            drive.followTrajectorySequence(RIGHTraj);
        }
        if (tagOfInterest.id==MIDDLE){
            TrajectorySequence MIDDLETraj = drive.trajectorySequenceBuilder(trajSeq8.end())
                    .lineToLinearHeading(new Pose2d(53, 0, Math.toRadians(90)))
                    .build();
            drive.followTrajectorySequence(MIDDLETraj);
        }
        else{
            TrajectorySequence LEFTTraj = drive.trajectorySequenceBuilder(trajSeq8.end())
                    .lineToLinearHeading(new Pose2d(53, -20, Math.toRadians(90)))
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
