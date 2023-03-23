package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.Aton.LimitSwitch;

public class PantherArm extends Thread{
    private Arm arm;
    private LimitSwitch armLimit;
    private LimitSwitch coneLimit;

    private LimitSwitch clawLimit;

    public PantherArm(Arm arm, LimitSwitch armLimit, LimitSwitch coneLimit,LimitSwitch clawLimit) {
        this.arm = arm;
        this.armLimit = armLimit;
        this.coneLimit = coneLimit;
        this.clawLimit = clawLimit;
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
        //while (arm.getArmMotorTargetPosition()!=arm.getArmMotorCurrentPosition()) {
            //let arm move
        //}
        sleep(75);
        arm.openGripper();
        sleep(100);
        arm.moveToTick(1400);
        //while (arm.getArmMotorTargetPosition()!=arm.getArmMotorCurrentPosition()) {
            //let arm move
        //}
    }
    public void grabCone() throws InterruptedException {
        arm.closeGripper();
        while (!armLimit.isPressed()) {
            arm.move(-1);

            if (!clawLimit.isPressed()){
                arm.move(0);
                arm.openGripper();
                sleep(75);
                arm.setTick(arm.getArmMotorCurrentPosition() + 570);
                break;
            }
        }

    }

    public void grabConeAton(){
        arm.closeGripper();
        while (!armLimit.isPressed()) {
            arm.move(-1);

            if (!clawLimit.isPressed() || (arm.getArmMotorCurrentPosition() <250)){
                arm.move(0);
                arm.openGripper();
                arm.setTick(arm.getArmMotorCurrentPosition() + 570);
                break;
            }
        }

    }
}
