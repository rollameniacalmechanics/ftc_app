package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by spmce on 12/2/2017.
 */

public class Warning {

    //HardwareMap hardwareMap;
    Telemetry telemetry;
    String message;

    public Warning(Telemetry telemetry) {
        //this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void addWarning(String message) {
        this.message += message;
    }

    public void displayMessage() {
        telemetry.addData("Warning: ", message);
    }
}
