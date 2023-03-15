package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Arm {
    private DcMotor armMotor;
    private Servo griper;
    private static int spoolInch = 2;
    private static final int tickRate = 28;

    static final double COUNTS_PER_INCH = (tickRate * 20) /
            (spoolInch * 3.1415);

    public Arm(HardwareMap hardwareMap, String armMotor, String griper) {
        this.griper = hardwareMap.get(Servo.class,griper);
        this.armMotor = hardwareMap.get(DcMotor.class,armMotor);
        this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void moveToTickT(int tick){
        new Thread(() -> {
            armMotor.setTargetPosition(tick);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(1);
            boolean timeout = true;
            int num = 0;
            while (armMotor.getTargetPosition() != armMotor.getCurrentPosition() && timeout) {
                //Wait for the arm to reach the target height
                if(num==100){
                    timeout=false;
                }
                num++;
            }
        }).start();
    }
    public void moveToTick(int tick){
            armMotor.setTargetPosition(tick);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(1);
            boolean timeout = true;
            int num = 0;
            while (armMotor.getTargetPosition() != armMotor.getCurrentPosition() && timeout) {
                //Wait for the arm to reach the target height
                if(num==50){
                    timeout=false;
                }
                num++;
            }
    }
    public void setTick(int tick){
        armMotor.setTargetPosition(tick);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
    }
    public void moveToHeight(double height) {
        int targetPosition = (int)(height /COUNTS_PER_INCH);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        while(armMotor.getTargetPosition() != armMotor.getCurrentPosition()) {
            //Wait for the arm to reach the target height
        }
        armMotor.setPower(0);
    }
    public void lowG() {
        int targetPosition = 1500;
        targetPosition = (int)((targetPosition/COUNTS_PER_INCH));
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        while(armMotor.getTargetPosition() != armMotor.getCurrentPosition()) {
            //Wait for the arm to reach the target height
        }
        armMotor.setPower(0);
    }
    public void medG() {
        int targetPosition = 2250;
        targetPosition = (int)((targetPosition*(spoolInch*Math.PI))/tickRate);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        while(armMotor.getTargetPosition() != armMotor.getCurrentPosition()) {
            //Wait for the arm to reach the target height
        }
        armMotor.setPower(0);
    }
    public void highG() {
        int targetPosition = 3000;
        targetPosition = (int)((targetPosition*(spoolInch*Math.PI))/tickRate);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        while(armMotor.getTargetPosition() != armMotor.getCurrentPosition()) {
            //Wait for the arm to reach the target height
        }
        armMotor.setPower(0);
    }
    public void coneHigh(int coneNum){
        double height = 5.5 * coneNum;
        int targetPosition = (int)(height / (spoolInch * Math.PI) * tickRate);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        while(armMotor.getTargetPosition() != armMotor.getCurrentPosition()) {
            //Wait for the arm to reach the target height
        }
        armMotor.setPower(0);
    }
    public void move(int power){
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(power);
    }
    public void runToPostion(){
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public double getGripperPos(){return griper.getPosition();}
    public void openGripper() {
        griper.setPosition(.5);
    }

    public void closeGripper() {
        griper.setPosition(1);
    }

    public int getArmMotorCurrentPosition(){
        return armMotor.getCurrentPosition();
    }
    public int getArmMotorTargetPosition(){
        return armMotor.getTargetPosition();
    }
    public void resetArmMotor(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
