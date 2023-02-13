package org.firstinspires.ftc.teamcode.drive.Aton;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimitSwitch {
    private DigitalChannel switchPin;
    public boolean defaultState;//the state of the switch when not pressed, vex switches default on/true

    public LimitSwitch(HardwareMap hardwareMap, String switchName,boolean defaultState) {
        switchPin = hardwareMap.get(DigitalChannel.class, switchName);
        switchPin.setMode(DigitalChannel.Mode.INPUT);
        this.defaultState = defaultState;

    }

    public boolean isPressed() {
        if(defaultState)
            return switchPin.getState();
        else
            return !switchPin.getState();
    }
}
