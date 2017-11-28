package org.firstinspires.ftc.teamcode.Components.GliphGrabber;

import org.firstinspires.ftc.teamcode.Components.DriveTrain.SwerveMotor;
import org.firstinspires.ftc.teamcode.Utilities.Map;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;

/**
 * Created by Tyler on 11/27/17.
 */

public class Grabber extends GliphGrabberHardware {
    // ------------------------ Constructor -------------------------
    /**
     * object used to map hardware
     */
    Map map;
    /**
     * object used to set Gliph Grabber power
     */
    SetRobot setRobot;

    /**
     * Constructs a one motor and one servo
     *
     * @param map object that is used to map the Gliph Grabber
     * @param setRobot object that is used to set the power to the Gliph Grabber
     */
    public Grabber(Map map, SetRobot setRobot) {
        this.map = map;
        this.setRobot = setRobot;
    }
    // -------------------------- Mapping ---------------------------
    /**
     * maps lift motor and crHand servo
     */
    @Override
    public void initHardware() {
        mLift  = map.motor("lift");
        crHand = map.revCrservo("crHand");

    }
    // --------------------- Set Hardware Power ---------------------
    /**
     * Sets power to lift motor and hand servo of drive train
     */
    @Override
    public void runHardware() {
        setRobot.power(mLift,liftPower,"lift motor");
        setRobot.position(crHand,crHandPosition,"hand crservo");

    }
}
/**
 *Tyler rocks
 */