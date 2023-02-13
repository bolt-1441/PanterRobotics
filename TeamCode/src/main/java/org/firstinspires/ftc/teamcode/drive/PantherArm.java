package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.Aton.LimitSwitch;

public class PantherArm{
    private Arm arm;
    private LimitSwitch armLimit;//the switch at the bottom of the arms movment
    private LimitSwitch coneLimit;//the switch inside the plow
    public PantherArm(Arm arm,LimitSwitch armLimit,LimitSwitch coneLimit){
        this.arm=arm;
        this.armLimit=armLimit;
        this.coneLimit=coneLimit;
    }
    public void armReset(){//brings the arm down 2,000 ticks until the armLimit switch is pressed,
                            //then the encoders for the arm motor are reset to help with encoder drift
        do{
            arm.moveToTick(arm.getArmMotorCurrentPosition()-2000);
            while (arm.getArmMotorTargetPosition()!=arm.getArmMotorCurrentPosition()){
                if (armLimit.isPressed())//lets the loop end early is the arm his the switch between the 2,000 ticks
                    break;
            }
        }while (!armLimit.isPressed());
        arm.resetArmMotor();
    }
}
