package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LineTracker {
    private AnalogInput lineTracker;

    public LineTracker(HardwareMap hardwareMap, String name) {
        lineTracker = hardwareMap.get(AnalogInput.class, name);
    }

    public double getVoltage() {
        return lineTracker.getVoltage();
    }

    public boolean isOnLine() {
        double voltage = getVoltage();
        return voltage < 1;
    }
}
