package org.firstinspires.ftc.teamcode.Components.JewelRejector;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.ComponentHardware;
import org.firstinspires.ftc.teamcode.Utilities.Map;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;

/**
 * Created by spmce on 11/18/2017.
 *
 * Jewel Rejector Component
 */
public class JewelRejector extends ComponentHardware {
    // ------------------------- Constants --------------------------
    public static final double BALL_PUSHER_UP = .44;
    public static final double BALL_PUSHER_DOWN = 1;
    public static final double BALL_ROTATOR_CENTER = .425;
    public static final double BALL_ROTATOR_RIGHT = .675;
    public static final double BALL_ROTATOR_LEFT = .175;
    // ------------ Standard Servos -------------
    public Servo ssBallPusher;
    public Servo ssBallRotator;
    // ---------------- Sensors -----------------
    public ColorSensor sColor;

    public double ballPusherPosition;
    public double ballRotatorPosition;

    public JewelRejector(Map map, SetRobot setRobot) {
        super();
        this.map = map;
        this.setRobot = setRobot;
        ssBallPusher = null;
        ssBallRotator = null;
        sColor = null;
        ballPusherPosition = BALL_PUSHER_UP;
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
        setRobot.position(ssBallPusher,ballPusherPosition,"ball pusher servo");
        setRobot.position(ssBallRotator,ballRotatorPosition,"ball rotator servo");
    }
}
