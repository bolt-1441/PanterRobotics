package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.Aton.LimitSwitch;

public class ArmThread extends Thread {
    private DcMotor arm;
    private LimitSwitch limitSwitch;

    public ArmThread(HardwareMap hardwareMap, String armName,LimitSwitch limitSwitch){
        this.arm = hardwareMap.get(DcMotor.class,armName);
        this.limitSwitch = limitSwitch;
    }

    @Override
    public void run() {
        while ( !(limitSwitch.isPressed())) {
            arm.setTargetPosition(arm.getCurrentPosition()-2000);
            while (arm.getCurrentPosition()!=arm.getTargetPosition()&&
                    !(limitSwitch.isPressed())) {
                if(limitSwitch.isPressed()) {
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    break;
                }
            }
        }
    }
}
