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
    }//voltage is the reflectivity detected by the sensor
    //0v-3.3v or 0v-6v

    public boolean isOnLine() {//boolean value for if the sensor is looking at something more reflective than the mats
        double voltage = getVoltage();
        return voltage < 1;
    }
}
