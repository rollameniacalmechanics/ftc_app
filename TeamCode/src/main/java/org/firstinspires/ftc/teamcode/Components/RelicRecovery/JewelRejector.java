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
    public static final double BALL_ROTATOR_CENTER = .425;
    /**
     * value of when the jewel rejector rotator is set right
     */
    public static final double BALL_ROTATOR_RIGHT = .675;
    /**
     * value of when the jewel rejector rotator is set left
     */
    public static final double BALL_ROTATOR_LEFT = .175;
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
    public double ballPusherPosition;
    public double ballRotatorPosition;

    public JewelRejector(Map map, SetRobot setRobot) {
        super();
        this.map = map;
        this.setRobot = setRobot;
        ssBallPusher = null;
        ssBallRotator = null;
        sColor = null;
        ballPusherPosition = JEWEL_REJECTOR_UP;
        ballRotatorPosition = BALL_ROTATOR_CENTER;
    }

    @Override
    public void initHardware() {
        ssBallPusher  = map.revServo("sBall",ballPusherPosition);
        ssBallRotator = map.servo("sBallRotator",ballRotatorPosition);
        sColor = map.colorSensor("cd");
    }

    @Override
    public void runHardware() {
        setRobot.position(ssBallPusher,ballPusherPosition,"jewel rejector servo");
        setRobot.position(ssBallRotator,ballRotatorPosition,"jewel rotator servo");
    }
}
