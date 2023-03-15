
package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
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

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;
import java.util.Queue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

@Autonomous(group = "drive")
public class testingClass extends LinearOpMode {
    private Servo wrist = null;
    private DcMotor turret = null;
    NormalizedColorSensor colorSensor;
    View relativeLayout;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);
        Arm arm = new Arm(hardwareMap,"turret","wrist");
        org.firstinspires.ftc.teamcode.drive.Aton.LimitSwitch coneDectc = new org.firstinspires.ftc.teamcode.drive.Aton.LimitSwitch(hardwareMap,"limitSwitchA",false);
        PantherArm pantherArm = new PantherArm(arm,new org.firstinspires.ftc.teamcode.drive.Aton.LimitSwitch(hardwareMap,"limitSwitch",false),
                coneDectc,new org.firstinspires.ftc.teamcode.drive.Aton.LimitSwitch(hardwareMap,"armConeDet",false));

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        wrist = hardwareMap.get(Servo.class,"wrist");
        turret = hardwareMap.get(DcMotor.class, "turret");
        LineTracker lineTrackerLeftForward = new LineTracker(hardwareMap,"lineTrackerLeftForward");
        LineTracker lineTrackerRightForward = new LineTracker(hardwareMap,"lineTrackerRightForward");
        LineTracker lineTrackerLeftBack = new LineTracker(hardwareMap,"lineTrackerRightBack");
        LineTracker lineTrackerRightBack = new LineTracker(hardwareMap,"lineTrackerLeftBack");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        runSample(); // actually execute the sample
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        wrist.setPosition(1);
        sleep(500);
        int val = 0;
        /////////////////////////THIS IS WHERE IT STARTS/////EVERYTHING BEFORE HERE IS INITIALIZATION/////////////
        waitForStart();
        sleep(50);//changed last
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        samePostitionarm();
        turret.setTargetPosition(700);
        turret.setPower(.7);
        samePostitionarm();
        sleep(500);
        int pos = 0;
        if (isStopRequested()) return;
        TrajectoryVelocityConstraint velConstraint1 = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(12)
        ));
        TrajectoryVelocityConstraint velConstraint2 = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(35)
        ));
        //Heads to cone and then reads the cone
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(velConstraint1)
                .splineToConstantHeading(new Vector2d(10,-6),Math.toRadians(0))
                .splineTo(new Vector2d(25,-6), Math.toRadians(0))
                .build();
        //Goes to the end of the field and lifts up the claw
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq.end())
                .splineTo(new Vector2d(57,-6),Math.toRadians(0))
                .addSpatialMarker(new Vector2d(57, -8), () -> {
                    // This marker runs at the point that gets
                    // closest to the (20, 20) coordinate
                    turret.setPower(1);
                    turret.setTargetPosition(1400);
                    samePostitionarm();
                    // Run your action in here!
                })
                .build();
        //Sets up for the cone placement
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(trajSeq2.end())
                .lineToLinearHeading(new Pose2d(49, -17, Math.toRadians(-180)))
                .build();
        //Heads to the cone stack
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(58,-21, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(53,-28, Math.toRadians(-90)))
                .build();

        drive.followTrajectorySequence(trajSeq);
        pos = runSample();
        sleep(150);
        drive.followTrajectorySequence(trajSeq2);
        drive.followTrajectorySequence(traj2);
        arm.moveToTickT(1100);
        //turret.setTargetPosition(1100);
        samePostitionarm();
        sleep(400);
        wrist.setPosition(.5);
        turret.setPower(1);
        turret.setTargetPosition(1200);
        samePostitionarm();
        drive.followTrajectorySequence(traj3);
        if(lineTrackerLeftBack.isOnLine()||lineTrackerLeftForward.isOnLine())
            val+=2;
        if (lineTrackerRightBack.isOnLine()||lineTrackerRightForward.isOnLine())
            val-=2;
        else
            val=0;
        telemetry.addData("val:",val);
        telemetry.update();
        // This marker runs at the point that gets
        // closest to the (20, 20) coordinate
        // Run your action in here!
        TrajectorySequence compensate = drive.trajectorySequenceBuilder(traj3.end())
                .setVelConstraint(velConstraint1)
                .lineToLinearHeading(new Pose2d(53+val,-38, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(53+val,-37, Math.toRadians(-90)))
                .build();
        //heads to place cone
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(compensate.end())
                .lineToLinearHeading(new Pose2d(52, -17, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(49, -17, Math.toRadians(-180)))
                .addSpatialMarker(new Vector2d(52, -20), () -> {
                    // This marker runs at the point that gets
                    // closest to the (20, 20) coordinate
                    turret.setTargetPosition(1400);
                    samePostitionarm();
                    // Run your action in here!
                })
                .build();
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(58,-21, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(53,-28, Math.toRadians(-90)))
                .build();
        //TODO MORE THAN ONE CONE PLACEMENT
        TrajectorySequence trajL = drive.trajectorySequenceBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(53,15,Math.toRadians(-90)))
                .build();
        TrajectorySequence trajC = drive.trajectorySequenceBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(53,-7,Math.toRadians(-90)))
                .build();
        TrajectorySequence trajR = drive.trajectorySequenceBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(53,-35,Math.toRadians(-90)))
                .build();
        sleep(600);
        AtomicBoolean running = new AtomicBoolean(true);
        Thread thread = new Thread(() -> {
            while (running.get()) {
                while (!coneDectc.isPressed()) {
                }
                sleep(370);
                try {
                    pantherArm.grabTopCone();
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        });
        thread.start();
        drive.followTrajectorySequence(compensate);
        running.set(false);
        sleep(500);
        wrist.setPosition(1);
        turret.setPower(1);
        sleep(500);
        turret.setTargetPosition(1400);
        samePostitionarm();
        sleep(400);
        drive.followTrajectorySequence(traj4);
        turret.setTargetPosition(1950);
        samePostitionarm();
        wrist.setPosition(.5);
        sleep(200);
        turret.setTargetPosition(2000);
        samePostitionarm();
        sleep(1000);
        drive.followTrajectorySequence(traj5);
        turret.setTargetPosition(200);
        samePostitionarm();
        //Reads the cone and sets the claw down

        if(pos == 1){
            drive.followTrajectorySequence(trajL);
        }
        else if(pos == 2){
            drive.followTrajectorySequence(trajC);
        }
        else{
            drive.followTrajectorySequence(trajR);
        }
        telemetry.addData("pos",pos);
        telemetry.update();
        sleep(5000);
    }
    //HOW TO MAKE THE ROBOT MOE TO A SPECIFIC SPOT//
//    drive.followTrajectory(
//            drive.trajectoryBuilder(traj.end(), true)
//            .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//            .build()
//        );
    private void samePostitionarm(){
        if(turret.getTargetPosition() != turret.getCurrentPosition()){
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("Status", "Run Time: " + turret.getCurrentPosition());
            telemetry.update();
        }
    }
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
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
            }
        });
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
