package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.Utilities.Map;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;

/**
 * Created by spmce on 11/27/2017.
 *
 * Abstract class for component hardware
 */
public abstract class ComponentHardware implements Component {
    // ---------------------- Hardware Objects ----------------------
    /**
     * object used to map hardware
     */
    protected Map map;
    /**
     * object used to set component power
     */
    protected SetRobot setRobot;
    // ------------------------ Constructor -------------------------
    /**
     * initializes objects to null and variables to 0
     */
    public ComponentHardware() {
        map      = null;
        setRobot = null;
    }
}
