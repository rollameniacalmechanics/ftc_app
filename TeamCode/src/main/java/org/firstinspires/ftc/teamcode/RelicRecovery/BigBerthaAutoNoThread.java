package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Components.RelicRecovery.GlyphGrabber;
import org.firstinspires.ftc.teamcode.Components.RelicRecovery.JewelRejector;
import org.firstinspires.ftc.teamcode.Utilities.ReadColor;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;
import org.firstinspires.ftc.teamcode.Utilities.UseIMU;
import org.firstinspires.ftc.teamcode.Utilities.UseVuforia;

import javax.microedition.khronos.opengles.GL;


/**
 * Created by Shane on 12/1/2017.
 */
public class BigBerthaAutoNoThread {

    HardwareMap hardwareMap;
    Telemetry telemetry;

    // do thread to continue after 5 seconds and open grabber after 28 seconds
    private BigBertha robot;

    /**
     * Encoders:
     * 60:1 1680
     * 40:1 1120
     * 20:1 560
     */
    private final static double COUNTS_PER_REVOLUTION = 1120.0; // 40:1 motor
    private final static double GEAR_ONE = 3.0;
    private final static double GEAR_TWO = 2.0;
    private final static double DIAMETER_OF_WHEEL = 4.0;

    private final static double RATIO = GEAR_ONE/GEAR_TWO;
    private final static double CIRCUMFERENCE_OF_WHEEL = DIAMETER_OF_WHEEL*Math.PI;
    private final static double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION/RATIO/CIRCUMFERENCE_OF_WHEEL;

    private final static double MAX_SPEED = 0.55;
    private final static double TURN_SPEED = .25 ;
    private final static double MIN_SPEED = .28;
    private final static double MIN_TURN_SPEED = .18;

    //UseVuforia useVuforia;
    ReadColor readColor;
    SetRobot setRobot;
    UseIMU useIMU;

    String jewelStatus = "waiting";
    String handStauts = "closed";
    String vuMark = "something";
    String myMessage = "";

    public BigBerthaAutoNoThread(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    private boolean oneDone = false;

    private States _state;

    public void initAuto(boolean ifFull) {
        _state = States.DOWN;
        // part 1
        robot = new BigBertha(hardwareMap,telemetry);
        robot.init();

        robot.glyphGrabber.crHand.setPower(-1);
        robot.driveTrain.mLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain.mRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.driveTrain.mLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveTrain.mRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        readColor = new ReadColor(robot.jewelRejector.sColor);

        // part 2
        setRobot = new SetRobot(telemetry);

        if (ifFull) {

            // part 3
            //useVuforia = new UseVuforia(hardwareMap, telemetry);
            //useVuforia.init();

            // part 4
            useIMU = new UseIMU(hardwareMap, telemetry);
            useIMU.init();

        }

        oneDone = true;
    }

    public void init_loop(boolean ifFull) {
        String status;
        if (oneDone) {
            status = "all done";
        } else {
            status = "not done";
        }
        telemetry.addData("status", status);
    }

    public void start() {
        //useVuforia.start();
        useIMU.start();
    }

    enum States {
        DOWN,
        WAIT_TO_READ,
        READING_VALUES,
        HIT_JEWEL,
        WAIT_TO_MOVE,
        MOVE_OFF_PLATE,
        ROTATE,
        RESET_ENCODERS,
        TO_BOX,
        RELEASE,
        STOP
    }

    private ReadColor.Color jewelColor = ReadColor.Color.NEITHER;
    private double myTime;

    public void loop(final boolean ifFull, final boolean ifBlue, final  boolean ifSide) {
        robot.run();
        if (ifFull) {
            useIMU.run();
        }
        switch(_state) {
            case DOWN:
                robot.jewelRejector.jewelRejectorPosition = JewelRejector.JEWEL_REJECTOR_DOWN;
                _state = States.WAIT_TO_READ;
                break;
            case WAIT_TO_READ:
                try {
                    Thread.sleep(300); // .1 second
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                }
                _state = States.READING_VALUES;
                break;
            case READING_VALUES:
                if(/*useVuforia.run() && */ readColor.readColor()) {
                    _state = States.HIT_JEWEL;
                }
                robot.jewelRejector.jewelRejectorPosition -= .005;
                //if()
                break;
            case HIT_JEWEL:
                jewelColor = readColor.getColorDetected();
                if (ifBlue) {
                    if (jewelColor == ReadColor.Color.RED) {
                        robot.jewelRejector.jewelRotatorPosition = JewelRejector.JEWEL_ROTATOR_RIGHT;
                    } else if (jewelColor == ReadColor.Color.BLUE) {
                        robot.jewelRejector.jewelRotatorPosition = JewelRejector.JEWEL_ROTATOR_LEFT;
                    } else {
                        telemetry.addData("Color", "neither, you messed up");
                    }
                } else {
                    if (jewelColor == ReadColor.Color.RED) {
                        robot.jewelRejector.jewelRotatorPosition = JewelRejector.JEWEL_ROTATOR_LEFT;
                    } else if (jewelColor == ReadColor.Color.BLUE) {
                        robot.jewelRejector.jewelRotatorPosition = JewelRejector.JEWEL_ROTATOR_RIGHT;
                    } else {
                        telemetry.addData("Color", "neither, you messed up");
                    }
                }
                if (ifFull) {
                    _state = States.WAIT_TO_MOVE;
                } else {
                    _state = States.STOP;
                }
                break;
            case WAIT_TO_MOVE:
                jewelStatus = "done";
                try {
                    Thread.sleep(275); // .1 second
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                }
                _state = States.MOVE_OFF_PLATE;
                break;
            case MOVE_OFF_PLATE:
                robot.jewelRejector.jewelRejectorPosition = JewelRejector.JEWEL_REJECTOR_UP;
                robot.jewelRejector.jewelRotatorPosition = JewelRejector.JEWEL_ROTATOR_CENTER;
                double distance;
                if(ifSide){
                    distance = 35.5;
                } else {
                    distance = 32.5;
                }
                robot.driveTrain.rightPower = MAX_SPEED;
                robot.driveTrain.leftPower = MAX_SPEED;
                if (robot.jewelRejector.sRange.getDistance(DistanceUnit.INCH)  >= distance) {
                    robot.driveTrain.stopHardware();
                    robot.driveTrain.leftPower = 0;
                    robot.driveTrain.rightPower = 0;
                    robot.driveTrain.mLeft.setPower(0);
                    robot.driveTrain.mRight.setPower(0);
                    robot.driveTrain.mBackLeft.setPower(0);
                    robot.driveTrain.mBackRight.setPower(0);
                    _state = States.ROTATE;
                }
                break;
            case ROTATE:
                double angle;
                if(ifSide){
                    /*if (useVuforia.getVuMark() == RelicRecoveryVuMark.RIGHT) {
                        angle = 104;
                        vuMark = "RIGHT";
                    } else if (useVuforia.getVuMark() == RelicRecoveryVuMark.LEFT) {
                        angle = 114;
                        vuMark = "LEFT";
                    } else {*/
                        angle = 90;
                        vuMark = "CENTER MAYBE";
                    //}
                } else {
                    angle = 95;
                }
                double leftPower = -TURN_SPEED;
                double rightPower = TURN_SPEED;
                if (ifBlue) {
                    robot.driveTrain.leftPower = leftPower;
                    robot.driveTrain.rightPower = rightPower;
                    myMessage = "ehhhhhhhhhhhhhhh";

                    if(useIMU.getHeading() >= (angle - 30)) {
                        leftPower = -MIN_TURN_SPEED;
                        rightPower = MIN_TURN_SPEED;
                        myMessage = "slowwwwwwwwwwwwwww";
                        if (useIMU.getHeading() >= angle) {
                            robot.driveTrain.stopHardware();
                            robot.driveTrain.leftPower = 0;
                            robot.driveTrain.rightPower = 0;
                            robot.driveTrain.mLeft.setPower(0);
                            robot.driveTrain.mRight.setPower(0);
                            robot.driveTrain.mBackLeft.setPower(0);
                            robot.driveTrain.mBackRight.setPower(0);
                            _state = States.RESET_ENCODERS;
                        }
                    }
                    robot.driveTrain.leftPower = leftPower;
                    robot.driveTrain.rightPower = rightPower;
                    robot.driveTrain.mLeft.setPower(leftPower);
                    robot.driveTrain.mRight.setPower(rightPower);
                    robot.driveTrain.mBackLeft.setPower(leftPower);
                    robot.driveTrain.mBackRight.setPower(rightPower);
                } else {
                    robot.driveTrain.leftPower = rightPower;
                    robot.driveTrain.rightPower = leftPower;
                    if(useIMU.getHeading() <= (-angle + 30)) {
                        leftPower = -MIN_TURN_SPEED;
                        rightPower = MIN_TURN_SPEED;
                        myMessage = "slowwwwwwwwwwwwwww";
                        if (useIMU.getHeading() <= angle) {
                            robot.driveTrain.stopHardware();
                            robot.driveTrain.leftPower = 0;
                            robot.driveTrain.rightPower = 0;
                            robot.driveTrain.mLeft.setPower(0);
                            robot.driveTrain.mRight.setPower(0);
                            robot.driveTrain.mBackLeft.setPower(0);
                            robot.driveTrain.mBackRight.setPower(0);
                            _state = States.RESET_ENCODERS;
                        }
                    }
                    robot.driveTrain.leftPower = rightPower;
                    robot.driveTrain.rightPower = leftPower;
                    robot.driveTrain.mLeft.setPower(rightPower);
                    robot.driveTrain.mRight.setPower(leftPower);
                    robot.driveTrain.mBackLeft.setPower(rightPower);
                    robot.driveTrain.mBackRight.setPower(leftPower);
                }
                break;
            case RESET_ENCODERS:
                robot.driveTrain.mLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.driveTrain.mRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.driveTrain.mLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.driveTrain.mRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                _state = States.TO_BOX;
                break;
            case TO_BOX:
                //double distance;
                //double rightPower;
                //double leftPower;
                if(ifSide){
                    rightPower = MAX_SPEED;
                    leftPower = MAX_SPEED;
                } else {
                    rightPower= 1;
                    leftPower = 0.35;
                }
                if (ifBlue) {
                    robot.driveTrain.mLeft.setPower(leftPower);
                    robot.driveTrain.mRight.setPower(rightPower);
                    robot.driveTrain.mBackLeft.setPower(rightPower);
                    robot.driveTrain.mBackRight.setPower(leftPower);
                    robot.driveTrain.leftPower = leftPower;
                    robot.driveTrain.rightPower = rightPower;
                } else {
                    double temp;
                    temp = rightPower;
                    rightPower = leftPower;
                    leftPower = temp;
                    robot.driveTrain.mLeft.setPower(leftPower);
                    robot.driveTrain.mRight.setPower(rightPower);
                    robot.driveTrain.mBackLeft.setPower(rightPower);
                    robot.driveTrain.mBackRight.setPower(leftPower);
                    robot.driveTrain.leftPower = leftPower;
                    robot.driveTrain.rightPower = rightPower;
                }
                if(ifSide){
                    distance = 24;
                } else {
                    distance = 49;
                }
                if (robot.driveTrain.mLeft.getCurrentPosition() >= (distance)*COUNTS_PER_INCH) {
                    robot.driveTrain.stopHardware();
                    robot.driveTrain.leftPower = 0;
                    robot.driveTrain.rightPower = 0;
                    robot.driveTrain.mLeft.setPower(0);
                    robot.driveTrain.mRight.setPower(0);
                    robot.driveTrain.mBackLeft.setPower(0);
                    robot.driveTrain.mBackRight.setPower(0);
                    _state = States.RELEASE;
                }
                break;
            case RELEASE:
                robot.glyphGrabber.crHandPosition = GlyphGrabber.HAND_OPEN;
                _state = States.STOP;
                break;
            case STOP:
                robot.driveTrain.stopHardware();
                robot.driveTrain.leftPower = 0;
                robot.driveTrain.rightPower = 0;
                robot.driveTrain.mLeft.setPower(0);
                robot.driveTrain.mRight.setPower(0);
                robot.driveTrain.mBackLeft.setPower(0);
                robot.driveTrain.mBackRight.setPower(0);
                break;
            default:
                telemetry.addData("Test", "Cry");
        }


        telemetry.addData("Right",robot.driveTrain.rightPower);
        telemetry.addData("Left",robot.driveTrain.leftPower);
        telemetry.addData("State",_state);
        telemetry.addData("VuMark",vuMark);
        //telemetry.addData("in", "%.2f in", range.getDistance(DistanceUnit.INCH));
        telemetry.addData("distance", robot.jewelRejector.sRange.getDistance(DistanceUnit.INCH));
        telemetry.addData("blue",robot.jewelRejector.sColor.blue());
        telemetry.addData("red",robot.jewelRejector.sColor.red());
        telemetry.addData("right encoder",robot.driveTrain.mRight.getCurrentPosition());
        telemetry.addData("left encoder",robot.driveTrain.mLeft.getCurrentPosition());
        telemetry.addData("arm lift encoder",robot.relicRetriever.mArmLift.getCurrentPosition());
        telemetry.addData("lift encoder",robot.glyphGrabber.mLift.getCurrentPosition());
        telemetry.addData("jewel", jewelStatus);
        telemetry.addData("hand", handStauts);
    }

    public void stop() {
        robot.glyphGrabber.crHand.setPower(GlyphGrabber.HAND_STOPPED);
        robot.glyphGrabber.crHandPosition = GlyphGrabber.HAND_STOPPED;
    }
}
