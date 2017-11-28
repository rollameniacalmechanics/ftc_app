package org.firstinspires.ftc.teamcode.SeasonCode.RelicRecoveryAlpha;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
<<<<<<< HEAD
import org.firstinspires.ftc.teamcode.Components.GliphGrabber.GliphGrabber;
import org.firstinspires.ftc.teamcode.Components.GliphGrabber.GliphGrabberHardware;
import org.firstinspires.ftc.teamcode.Components.GliphGrabber.Grabber;
=======
import org.firstinspires.ftc.teamcode.Components.RelicRetriever.RelicRetriever;
import org.firstinspires.ftc.teamcode.Components.RelicRetriever.RelicRetrieverHardware;
import org.firstinspires.ftc.teamcode.Components.RelicRetriever.Retriever;
>>>>>>> origin/master

import static org.firstinspires.ftc.teamcode.Utilities.ServoPositions.ARM_IN;
import static org.firstinspires.ftc.teamcode.Utilities.ServoPositions.BALL_PUSHER_UP;
import static org.firstinspires.ftc.teamcode.Utilities.ServoPositions.BALL_ROTATOR_CENTER;
import static org.firstinspires.ftc.teamcode.Utilities.ServoPositions.GRABBER_CLOSED;
import static org.firstinspires.ftc.teamcode.Utilities.ServoPositions.HAND_STOPPED;

/**
 * Created by Shane on 26-11-2017.
 */
public class BigBerthaRelicRecoveryRobot extends Robot {
<<<<<<< HEAD
    GliphGrabberHardware gliphGrabber;
    // ---------------------- Hardware Devices ----------------------
    // ---------------- DcMotors ----------------
    //protected DcMotor mLift        = null;
    protected DcMotor mArmLift     = null;
=======
    RelicRetrieverHardware relicRetriever;
    // ---------------------- Hardware Devices ----------------------
    // ---------------- DcMotors ----------------
    protected DcMotor mLift        = null;
    //protected DcMotor mArmLift     = null;
>>>>>>> origin/master
    // ------------ Standard Servos -------------
    //private Servo ssArm          = null;
    //private Servo ssRelicGrabber = null;
    private Servo ssBallPusher   = null;
    private Servo ssBallRotator  = null;
    // ------- Continuous Rotation Servos -------
    //private CRServo crHand       = null;
    // ---------------- Sensors -----------------
    public ColorSensor sColor;
    // --------------------- Hardware Variables ---------------------
    // ---------------- DcMotors ----------------
<<<<<<< HEAD
    //public double liftPower;
    public double armLiftPower;
=======
    public double liftPower;
    //public double armLiftPower;
>>>>>>> origin/master
    // ------------ Standard Servos -------------
    //public double armPosition;
    //public double grabberPosition;
    public double ballPusherPosition;
    public double ballRotatorPosition;
    // ------- Continuous Rotation Servos -------
    //public double crHandPosition;
    // ------------------------ Constructor -------------------------
    BigBerthaRelicRecoveryRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap,telemetry);
<<<<<<< HEAD
        gliphGrabber = new Grabber(map,setRobot);
        //liftPower           = 0;
        armLiftPower        = 0;
        armPosition         = ARM_IN;
        grabberPosition     = GRABBER_CLOSED;
=======
        relicRetriever = new Retriever(map, setRobot);
        liftPower           = 0;
        //armLiftPower        = 0;
        //armPosition         = ARM_IN;
        //grabberPosition     = GRABBER_CLOSED;
>>>>>>> origin/master
        ballPusherPosition  = BALL_PUSHER_UP;
        ballRotatorPosition = BALL_ROTATOR_CENTER;
        //crHandPosition      = HAND_STOPPED;
    }
    // -------------------------- Mapping ---------------------------
    void mapMotors() {
        relicRetriever.initHardware();
        // -------------- DcMotors --------------
<<<<<<< HEAD
        gliphGrabber.initHardware();
        //mLift          = map.motor("lift");
        mArmLift       = map.motor("armLift");
=======
        mLift          = map.motor("lift");
        //mArmLift       = map.motor("armLift");
>>>>>>> origin/master
    }
    void mapServos() {
        // ---------- Standard Servos -----------
        //ssArm          = map.revServo("sArm", armPosition);
        //ssRelicGrabber = map.servo("sGrabber", grabberPosition);
        ssBallPusher   = map.servo("sBall", ballPusherPosition);
        ssBallRotator  = map.servo("sBallRotator", ballRotatorPosition);
    }
    void mapCRServos() {
        // ----- Continuous Rotation Servos -----
        //crHand         = map.revCrservo("crHand");
    }
    void mapSensors() {
        // -------------- Sensors ---------------
        sColor         = map.colorSensor("cd");
    }
    // --------------------- Set Hardware Power ---------------------
    void setMotorPowers() {
<<<<<<< HEAD
        gliphGrabber.runHardware();
        // -------------- DcMotors --------------
        //setRobot.power(mLift,liftPower,"lift motor");
        setRobot.power(mArmLift,armLiftPower,"arm lift motor");
=======
        relicRetriever.runHardware();
        // -------------- DcMotors --------------
        setRobot.power(mLift,liftPower,"lift motor");
        //setRobot.power(mArmLift,armLiftPower,"arm lift motor");
>>>>>>> origin/master
    }
    void setServoPositions() {
        // ---------- Standard Servos -----------
        //setRobot.position(ssArm,armPosition,"arm servo");
        //setRobot.position(ssRelicGrabber,grabberPosition,"relic grabber servo");
        setRobot.position(ssBallPusher,ballPusherPosition,"ball pusher servo");
        setRobot.position(ssBallRotator, ballRotatorPosition,"ball rotator servo");
    }
    void setCRServoPowers() {
        // ----- Continuous Rotation Servos -----
        //setRobot.position(crHand,crHandPosition,"hand crservo");
    }
}
