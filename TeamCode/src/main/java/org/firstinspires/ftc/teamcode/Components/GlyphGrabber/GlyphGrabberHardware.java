package org.firstinspires.ftc.teamcode.Components.GlyphGrabber;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utilities.Map;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;

/**
 * Created by Tyler on 11/27/17.
 *
 * Abstract class for drive train Hardware
 * Contains left and right motors and variables
 */
public abstract class GlyphGrabberHardware implements GlyphGrabber {
    // ---------------------- Hardware Objects ----------------------
    /**
     * object used to map hardware
     */
    protected Map map;
    /**
     * object used to set Glyph Grabber power
     */
    protected SetRobot setRobot;
    // ---------------------- Hardware Devices ----------------------

    // ------------------------ Constructor -------------------------
    /**
     * constructor / inti objects to null and variables to 0
     */
    GlyphGrabberHardware() {
        map         = null;
        setRobot    = null;

    }
}
/**
 *Tyler rocks
 */