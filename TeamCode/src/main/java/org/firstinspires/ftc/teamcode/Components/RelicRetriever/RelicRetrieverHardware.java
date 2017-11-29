package org.firstinspires.ftc.teamcode.Components.RelicRetriever;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.Map;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;

/**
 * Created by jeppe on 27-11-2017.
 *
 * Abstract class for drive train Hardware
 * Contains left and right motors and variables
 */

public abstract class RelicRetrieverHardware implements RelicRetriever {
    // ---------------------- Hardware Objects ----------------------
    /**
     * object used to map hardware
     */
    protected Map map;
    /**
     * object used to set drive train power
     */
    protected SetRobot setRobot;
    // ---------------------- Hardware Devices ----------------------

    // ------------------------ Constructor -------------------------
    public RelicRetrieverHardware(){

    }
}
