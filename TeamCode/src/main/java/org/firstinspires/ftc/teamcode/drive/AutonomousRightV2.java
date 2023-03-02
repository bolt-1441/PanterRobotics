
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Aton.LimitSwitch;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(group = "drive")
public class AutonomousRightV2 extends LinearOpMode {
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
        LimitSwitch coneDectc = new LimitSwitch(hardwareMap,"limitSwitchA",false);
        PantherArm pantherArm = new PantherArm(arm,new LimitSwitch(hardwareMap,"limitSwitch",false),
                coneDectc);
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        wrist = hardwareMap.get(Servo.class,"wrist");
        wrist.setDirection(Servo.Direction.REVERSE);
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
        colorSensor.getNormalizedColors();
        wrist.setPosition(1);
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
        sleep(300);
        int pos;
        if (isStopRequested()) return;
        TrajectoryVelocityConstraint velConstraint1 = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(17)
        ));
        TrajectoryVelocityConstraint velConstraint2 = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(45)
        ));
        //Heads to cone and then reads the cone
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(velConstraint1)
                .splineToConstantHeading(new Vector2d(10,-7),Math.toRadians(0))
                .splineTo(new Vector2d(27,-9), Math.toRadians(0))
                .build();
        //Sets up for the cone placement
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(trajSeq.end())
                .splineTo(new Vector2d(57,-8),Math.toRadians(0))
                .addSpatialMarker(new Vector2d(57, -8), () -> {
                    // This marker runs at the point that gets
                    // closest to the (20, 20) coordinate
                    turret.setPower(1);
                    turret.setTargetPosition(1400);
                    samePostitionarm();
                    // Run your action in here!
                })
                .addSpatialMarker(new Vector2d(55 -10), () -> {
                    // This marker runs at the point that gets
                    arm.moveToTickT(1300);
                })
                .lineToLinearHeading(new Pose2d(49, -18, Math.toRadians(-180)))
                .addSpatialMarker(new Vector2d(40, -18), () -> {
                    // This marker runs at the point that gets
                    // closest to the (20, 20) coordinate
                    wrist.setPosition(.5);
                    // Run your action in here!
                })
                .lineToLinearHeading(new Pose2d(58,-21, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(53,-28, Math.toRadians(-90)))
                .build();
        //Heads to the cone stack
        drive.followTrajectorySequence(trajSeq);
        pos = runSample();
        sleep(10);
        requestOpModeStop();
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
                .lineToLinearHeading(new Pose2d(55,3, Math.toRadians(-90)))
                .addSpatialMarker(new Vector2d(55, -35), () -> {
                    // This marker runs at the point that gets
                    // closest to the (20, 20) coordinate
                    arm.moveToTickT(400);
                    // Run your action in here!
                })
                .lineToLinearHeading(new Pose2d(57, 4, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(55, 4, Math.toRadians(-180)))
                .waitSeconds(.1)
                .lineToLinearHeading(new Pose2d(51, 5, Math.toRadians(-180)))
                .addSpatialMarker(new Vector2d(53, 2), () -> {
                    // This marker runs at the point that gets
                    // closest to the (20, 20) coordinate
                    arm.moveToTickT(2300);
                    // Run your action in here!
                })
                .lineToLinearHeading(new Pose2d(52, 3, Math.toRadians(-180)))
                .build();
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(62, 4, Math.toRadians(-180)))
                .build();
        //TODO MORE THAN ONE CONE PLACEMENT
        TrajectorySequence trajL = drive.trajectorySequenceBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(55,15,Math.toRadians(-90)))
                .build();
        TrajectorySequence trajC = drive.trajectorySequenceBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(58,-7,Math.toRadians(-90)))
                .build();
        TrajectorySequence trajR = drive.trajectorySequenceBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(62,-32,Math.toRadians(-90)))
                .build();
        sleep(200);
        AtomicBoolean running = new AtomicBoolean(true);
        Thread thread = new Thread(() -> {
                //while (!coneDectc.isPressed()){}
            ElapsedTime elapsedTime = new ElapsedTime();
            while (!coneDectc.isPressed()&& elapsedTime.seconds()<5) {
            }
            sleep(450);
            try {
                pantherArm.grabTopCone();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
           // }
        });
        thread.start();
        drive.followTrajectorySequence(compensate);
        running.set(false);
        sleep(100);
        wrist.setPosition(1);
        turret.setPower(1);
        sleep(100);
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
        sleep(500);
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
    private void samePostitionarm(){
        if(turret.getTargetPosition() != turret.getCurrentPosition()){
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("Status", "Run Time: " + turret.getCurrentPosition());
            telemetry.update();
        }
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
