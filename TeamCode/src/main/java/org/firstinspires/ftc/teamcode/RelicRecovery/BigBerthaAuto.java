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


/**
 * Created by Shane on 12/1/2017.
 */
public class BigBerthaAuto {

    HardwareMap hardwareMap;
    Telemetry telemetry;

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

    private final static double MAX_SPEED = 1;
    private final static double TURN_SPEED = .5;
    private final static double MIN_SPEED = .28;

    UseVuforia useVuforia;
    ReadColor readColor;
    SetRobot setRobot;
    UseIMU useIMU;

    String jewelStatus = "waiting";
    String handStauts = "closed";
    String vuMark = "something";

    public BigBerthaAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    private boolean oneDone = false;
    private boolean twoDone = false;
    private boolean threeDone = false;
    private boolean fourDone = false;

    private States _state;

    public void initAuto(boolean ifFull) {
        _state = States.DOWN;
        Thread t = new Thread(new Runnable() {
            @Override
            public void run() {
                robot = new BigBertha(hardwareMap,telemetry);
                robot.init();

                robot.glyphGrabber.crHand.setPower(-1);
                robot.driveTrain.mLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.driveTrain.mRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.driveTrain.mLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.driveTrain.mRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                readColor = new ReadColor(robot.jewelRejector.sColor);
                oneDone = true;
            }
        });
        Thread u = new Thread(new Runnable() {
            @Override
            public void run() {
                setRobot = new SetRobot(telemetry);
                twoDone = true;
            }
        });
        t.start();
        u.start();
        if (ifFull) {
            Thread v = new Thread(new Runnable() {
                @Override
                public void run() {
                    useVuforia = new UseVuforia(hardwareMap, telemetry);
                    useVuforia.init();
                    threeDone = true;
                }
            });
            Thread w = new Thread(new Runnable() {
                @Override
                public void run() {
                    useIMU = new UseIMU(hardwareMap, telemetry);
                    useIMU.init();
                    fourDone = true;
                }
            });
            v.start();
            w.start();
            _state = States.TIMER;
        }
    }

    public void init_loop(boolean ifFull) {
        String status;
        if (!ifFull || (threeDone && fourDone)) {
            if (oneDone && twoDone) {
                status = "all done";
            } else {
                status = "not done";
            }
        } else {
            status = "not done!";
        }
        telemetry.addData("status", status);
    }

    public void start() {
        useVuforia.start();
        useIMU.start();
    }

    enum States {
        TIMER,
        DOWN,
        WAIT_TO_READ,
        READ_PICTURE,
        READING_VALUES,
        HIT_JEWEL,
        WAIT_TO_MOVE,
        MOVE_OFF_PLATE,
        ROTATE,
        WAIT,
        RESET_ENCODERS,
        TO_BOX,
        ROTATE_TO_BOX,
        RESET_ENCODERS_AGAIN,
        TO_BOX_AGAIN,
        STOP
    }

    private ReadColor.Color jewelColor = ReadColor.Color.NEITHER;

    public void loop(final boolean ifFull, final boolean ifBlue, final  boolean ifSide) {
        robot.run();
        if (ifFull) {
            useIMU.run();
        }
        switch(_state) {
            case TIMER:
                robot.jewelRejector.jewelRejectorPosition = JewelRejector.JEWEL_REJECTOR_DOWN;
                Thread a = new Thread(new Runnable() {
                    @Override
                    public void run() {
                        try {
                            Thread.sleep(21000); // 21 seconds
                        } catch (InterruptedException ex) {
                            Thread.currentThread().interrupt();
                        }
                        robot.glyphGrabber.crHand.setPower(GlyphGrabber.HAND_OPEN);
                        robot.glyphGrabber.crHandPosition = GlyphGrabber.HAND_OPEN;
                        handStauts = "open";
                        _state = States.WAIT;

                        try {
                            Thread.sleep(1100); // 1.1 seconds
                        } catch (InterruptedException ex) {
                            Thread.currentThread().interrupt();
                        }
                        robot.driveTrain.leftPower = -.5;
                        robot.driveTrain.rightPower = -.5;
                        robot.driveTrain.mRight.setPower(-.5);
                        robot.driveTrain.mLeft.setPower(-.5);

                        try {
                            Thread.sleep(400); // .4 seconds
                        } catch (InterruptedException ex) {
                            Thread.currentThread().interrupt();
                        }
                        robot.driveTrain.stopHardware();
                        _state = States.STOP;
                    }
                });
                Thread b = new Thread(new Runnable() {
                    @Override
                    public void run() {
                        try {
                            Thread.sleep(5000); // .1 second
                        } catch (InterruptedException ex) {
                            Thread.currentThread().interrupt();
                        }
                        robot.driveTrain.leftPower = .26;
                        robot.driveTrain.rightPower = .26;
                        try {
                            Thread.sleep(450); // .1 second
                        } catch (InterruptedException ex) {
                            Thread.currentThread().interrupt();
                        }
                        robot.driveTrain.leftPower = 0;
                        robot.driveTrain.rightPower = 0;
                        try {
                            Thread.sleep(500); // .1 second
                        } catch (InterruptedException ex) {
                            Thread.currentThread().interrupt();
                        }
                        if (_state == States.READING_VALUES) {
                            if (ifFull) {
                                _state = States.HIT_JEWEL;
                            }
                        }
                        jewelStatus = "skipped";
                    }
                });
                a.start();
                b.start();
                _state = States.DOWN;
                break;
            case READ_PICTURE:
                Thread f = new Thread(new Runnable() {
                    @Override
                    public void run() {
                        while (!useVuforia.run()) {
                            telemetry.addData("Vuforia", "READING NOW");
                        }
                    }
                });
                f.run();
                _state = States.DOWN;
                break;
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

                Thread s = new Thread(new Runnable() {
                    @Override
                    public void run() {
                        double distance;
                        if(ifSide){
                            distance = 33.5;
                        } else {
                            distance = 32.5;
                        }
                        while (robot.jewelRejector.sRange.getDistance(DistanceUnit.INCH)  <= distance) {
                            robot.driveTrain.rightPower = MAX_SPEED;
                            robot.driveTrain.leftPower = MAX_SPEED;
                        }
                        robot.driveTrain.stopHardware();
                        _state = States.ROTATE;
                    }
                });
                s.start();
                _state = States.WAIT;
                break;
            case ROTATE:
                Thread t = new Thread(new Runnable() {
                    @Override
                    public void run() {
                        double angle;
                        if(ifSide){
                            if (useVuforia.getVuMark() == RelicRecoveryVuMark.RIGHT) {
                                angle = 104;
                                vuMark = "RIGHT";
                            } else if (useVuforia.getVuMark() == RelicRecoveryVuMark.LEFT) {
                                angle = 114;
                                vuMark = "LEFT";
                            } else {
                                angle = 85;
                                vuMark = "CENTER MAYBE";
                            }
                        } else {
                            angle = 95;
                        }
                        double leftPower = -TURN_SPEED;
                        double rightPower = TURN_SPEED;
                        if (ifBlue) {
                            robot.driveTrain.leftPower = leftPower;
                            robot.driveTrain.rightPower = rightPower;
                            while (useIMU.getHeading() < angle) {
                                if(useIMU.getHeading() >= angle - 30) {
                                    leftPower = -MIN_SPEED/3;
                                    rightPower = MIN_SPEED/3;
                                }
                                robot.driveTrain.leftPower = leftPower;
                                robot.driveTrain.rightPower = rightPower;
                            }
                        } else {
                            robot.driveTrain.leftPower = rightPower;
                            robot.driveTrain.rightPower = leftPower;
                            while (useIMU.getHeading() > -angle) {
                                if(useIMU.getHeading() <= -angle + 30) {
                                    leftPower = MIN_SPEED/3;
                                    rightPower = MIN_SPEED/3;
                                }
                                robot.driveTrain.leftPower = rightPower;
                                robot.driveTrain.rightPower = leftPower;
                            }
                        }
                        robot.driveTrain.stopHardware();
                        _state = States.RESET_ENCODERS;
                    }
                });
                t.start();
                _state = States.WAIT;
                break;
            case WAIT:
                telemetry.addData("Im waiting", "waiting");
                break;
            case RESET_ENCODERS:
                robot.driveTrain.mLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.driveTrain.mRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.driveTrain.mLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.driveTrain.mRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                _state = States.TO_BOX;
                break;
            case TO_BOX:
                Thread v = new Thread(new Runnable() {
                    @Override
                    public void run() {
                        double distance;
                        double rightPower;
                        double leftPower;
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
                            robot.driveTrain.leftPower = leftPower;
                            robot.driveTrain.rightPower = rightPower;
                        } else {
                            double temp;
                            temp = rightPower;
                            rightPower = leftPower;
                            leftPower = temp;
                            robot.driveTrain.mLeft.setPower(leftPower);
                            robot.driveTrain.mRight.setPower(rightPower);
                            robot.driveTrain.leftPower = leftPower;
                            robot.driveTrain.rightPower = rightPower;
                        }
                        if(ifSide){
                            distance = 15;
                        } else {
                            distance = 49;
                        }
                        while (true) {
                            if (robot.driveTrain.mLeft.getCurrentPosition() >= (distance)*COUNTS_PER_INCH)
                                break;
                        }
                        robot.driveTrain.stopHardware();

                        _state = States.STOP;
                    }
                });
                v.start();
                _state = States.WAIT;
                break;
            case ROTATE_TO_BOX:
                Thread u = new Thread(new Runnable() {
                    @Override
                    public void run() {
                        if (ifSide) {
                            if (ifBlue) {
                                robot.driveTrain.leftPower = .22;
                                robot.driveTrain.rightPower = -.22;
                                while (true) {
                                    if (useIMU.getHeading() <= 99)
                                        break;
                                }
                            } else {
                                robot.driveTrain.leftPower = -.22;
                                robot.driveTrain.rightPower = .22;
                                while (true) {
                                    if (useIMU.getHeading() >= -101)
                                        break;
                                }
                            }
                        } else {
                            if (ifBlue) {
                                robot.driveTrain.leftPower = -.5;
                                robot.driveTrain.rightPower = .5;
                                while (true) {
                                    if (useIMU.getHeading() > -170 && useIMU.getHeading() < -90)
                                        break;
                                }
                            } else {
                                robot.driveTrain.leftPower = .5;
                                robot.driveTrain.rightPower = -.5;
                                while (true) {
                                    if (useIMU.getHeading() < 170 && useIMU.getHeading() > 90)
                                        break;
                                }
                            }
                        }
                        robot.driveTrain.stopHardware();

                        _state = States.RESET_ENCODERS_AGAIN;
                    }
                });
                u.start();
                _state = States.WAIT;
                break;
            case RESET_ENCODERS_AGAIN:
                robot.driveTrain.mLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.driveTrain.mRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.driveTrain.mLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.driveTrain.mRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                _state = States.TO_BOX_AGAIN;
                break;
            case TO_BOX_AGAIN:
                robot.driveTrain.leftPower = .2;
                robot.driveTrain.rightPower = .2;
                double distance;
                if (ifSide) {
                    distance = 11;
                } else {
                    distance = 20;
                }
                if (robot.driveTrain.mLeft.getCurrentPosition() > (distance)*COUNTS_PER_INCH) {
                    _state = States.STOP;
                }
                break;
            case STOP:
                robot.driveTrain.stopHardware();
                robot.driveTrain.mLeft.setPower(0);
                robot.driveTrain.mRight.setPower(0);
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
