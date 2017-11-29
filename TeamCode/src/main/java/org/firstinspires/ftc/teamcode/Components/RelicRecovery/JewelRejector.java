package org.firstinspires.ftc.teamcode.Components.RelicRecovery;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.ComponentInterface.ComponentHardware;
import org.firstinspires.ftc.teamcode.Utilities.Map;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;

/**
 * Created by spmce on 11/18/2017.
 *
 * The Jewel Rejector Component is used to knock off the other team's color jewel
 */
public class JewelRejector extends ComponentHardware {
    // ------------------------- Constants --------------------------
    /**
     * value of when the jewel rejector is up
     */
    public static final double JEWEL_REJECTOR_UP = .44;
    /**
     * value of when the jewel rejector is down
     */
    public static final double JEWEL_REJECTOR_DOWN = 1;
    /**
     * value of when the jewel rejector rotator is centered
     */
    public static final double JEWEL_ROTATOR_CENTER = .425;
    /**
     * value of when the jewel rejector rotator is set right
     */
    public static final double JEWEL_ROTATOR_RIGHT = .675;
    /**
     * value of when the jewel rejector rotator is set left
     */
    public static final double JEWEL_ROTATOR_LEFT = .175;
    // ---------------------- Hardware Devices ----------------------
    // ------------ Standard Servos -------------
    /**
     * servo that pushes jewel
     */
    public Servo ssBallPusher;
    /**
     * servo that rotates the jewel rejector servo
     */
    public Servo ssBallRotator;
    // ---------------- Sensors -----------------
    /**
     * color sensor used to detect color of jewels
     */
    public ColorSensor sColor;
    // --------------------- Hardware Variables ---------------------
    /**
     * this variable is used to set power the jewel rejector servo
     */
    public double jewelRejectorPosition;
    /**
     * this variable is used to set power the jewel rejector rotator servo
     */
    public double jewelRotatorPosition;
    // ------------------------ Constructor -------------------------
    /**
     * Contructor that initializes hardware devices to null or default power
     *
     * @param map object that is used to map the jewel rejector components
     * @param setRobot object that is used to set the power to the jewel rejector components
     */
    public JewelRejector(Map map, SetRobot setRobot) {
        super();
        this.map = map;
        this.setRobot = setRobot;
        ssBallPusher = null;
        ssBallRotator = null;
        sColor = null;
        jewelRejectorPosition = JEWEL_REJECTOR_UP;
        jewelRotatorPosition = JEWEL_ROTATOR_CENTER;
    }
    // -------------------------- Mapping ---------------------------
    /**
     * maps jewel rejector components to phones
     */
    @Override
    public void initHardware() {
        ssBallPusher  = map.revServo("sBall", jewelRejectorPosition);
        ssBallRotator = map.servo("sBallRotator", jewelRotatorPosition);
        sColor = map.colorSensor("cd");
    }
    // --------------------- Set Hardware Power ---------------------
    /**
     * sets power to jewel rejector components
     */
    @Override
    public void runHardware() {
        setRobot.position(ssBallPusher, jewelRejectorPosition,"jewel rejector servo");
        setRobot.position(ssBallRotator, jewelRotatorPosition,"jewel rotator servo");
    }
}
