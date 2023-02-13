package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.Aton.LimitSwitch;

public class PantherArm {
    private Arm arm;
    private LimitSwitch armLimit;
    private LimitSwitch coneLimit;

    public PantherArm(Arm arm, LimitSwitch armLimit, LimitSwitch coneLimit) {
        this.arm = arm;
        this.armLimit = armLimit;
        this.coneLimit = coneLimit;
    }

    public void armReset() {
        while ( !(armLimit.isPressed())) {
            arm.moveToTick(arm.getArmMotorCurrentPosition()-2000);
            while (arm.getArmMotorTargetPosition()!=arm.getArmMotorCurrentPosition()&&
                    !(armLimit.isPressed())) {
                if(armLimit.isPressed()) {
                    arm.resetArmMotor();
                    break;
                }
            }
        }
        arm.resetArmMotor();
    }
}
