package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.Aton.LimitSwitch;

public class PantherArm extends Thread{
    private Arm arm;
    private LimitSwitch armLimit;
    private LimitSwitch coneLimit;

    public PantherArm(Arm arm, LimitSwitch armLimit, LimitSwitch coneLimit) {
        this.arm = arm;
        this.armLimit = armLimit;
        this.coneLimit = coneLimit;
    }

    public void armReset() {
        arm.closeGripper();
        while (!armLimit.isPressed())
        {
            arm.move(-1);
        }
        arm.move(0);
        arm.resetArmMotor();
        arm.runToPostion();
    }
    public void grabTopCone() throws InterruptedException {
        arm.closeGripper();
        arm.moveToTick(560);
        while (arm.getArmMotorTargetPosition()!=arm.getArmMotorCurrentPosition()) {
            //let arm move
        }
        sleep(75);
        arm.openGripper();
        sleep(85);
        arm.moveToTick(1400);
        while (arm.getArmMotorTargetPosition()!=arm.getArmMotorCurrentPosition()) {
            //let arm move
        }
    }
    public void grabCone(){
        new Thread(()-> {
            arm.closeGripper();
            arm.moveToTick(1);
            while (!armLimit.isPressed()) {
            }
            arm.openGripper();
            arm.moveToTick(350);
        }).start();

    }
}
