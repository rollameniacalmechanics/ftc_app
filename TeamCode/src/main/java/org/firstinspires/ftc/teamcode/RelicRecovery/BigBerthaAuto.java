package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.RelicRecovery.GlyphGrabber;
import org.firstinspires.ftc.teamcode.Components.RelicRecovery.JewelRejector;
import org.firstinspires.ftc.teamcode.Utilities.ReadColor;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;
import org.firstinspires.ftc.teamcode.Utilities.UseIMU;
import org.firstinspires.ftc.teamcode.Utilities.UseVuforia;


/**
 * Created by Shane on 12/1/2017.
 */
//@Autonomous(name = "Relic Recovery Auto Blue", group = "Autonomous")
public class BigBerthaAuto {

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

    UseVuforia useVuforia;
    ReadColor readColor;
    SetRobot setRobot;
    UseIMU useIMU;

    //ModernRoboticsI2cRangeSensor range = null;

    //VoltageSensor voltageSensor;

    String jewelStatus = "waiting";
    String handStauts = "closed";

    public BigBerthaAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    private int state = 0;
    private boolean ifDone = false;

    private boolean oneDone = false;
    private boolean twoDone = false;
    private boolean threeDone = false;
    private boolean fourDone = false;


    private States _state;

    public void initAuto(boolean ifFull) {
        //voltageSensor = hardwareMap.voltageSensor.get("voltage");
        _state = States.READING_VALUES;
        Thread t = new Thread(new Runnable() {
            @Override
            public void run()
            {
                robot = new BigBertha(hardwareMap,telemetry);
                //range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "r");
                robot.init();
                //range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "r");
                robot.glyphGrabber.crHand.setPower(-1);

                robot.driveTrain.mLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.driveTrain.mRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.driveTrain.mLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.driveTrain.mRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                readColor = new ReadColor(robot.jewelRejector.sColor);
                telemetry.addData("Robot Init","done");
                oneDone = true;
            }
        });
        Thread u = new Thread(new Runnable() {
            @Override
            public void run()
            {
                setRobot = new SetRobot(telemetry);
                telemetry.addData("Set Robot Init","done");
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
                    telemetry.addData("Use Vuforia Init", "done");
                    threeDone = true;
                }
            });
            Thread w = new Thread(new Runnable() {
                @Override
                public void run() {
                    useIMU = new UseIMU(hardwareMap, telemetry);
                    useIMU.init();
                    telemetry.addData("Use IMU Init", "done");
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
            status = "not done?";
        }
        telemetry.addData("status", status);
        //if (robot.jewelRejector.sRange != null)
                //telemetry.addData("distance", robot.jewelRejector.sRange.getDistance(DistanceUnit.INCH));
        /*if (range != null)
            telemetry.addData("range", range.getDistance(DistanceUnit.INCH));*/
    }

    public void start() {
        useVuforia.start();
        useIMU.start();
    }

    enum States {
        TIMER,
        WAIT_TO_READ,
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
                Thread a = new Thread(new Runnable() {
                    @Override
                    public void run()
                    {
                        try {
                            Thread.sleep(22000); // .1 second
                        } catch (InterruptedException ex) {
                            Thread.currentThread().interrupt();
                        }
                        robot.glyphGrabber.crHand.setPower(GlyphGrabber.HAND_OPEN);
                        robot.glyphGrabber.crHandPosition = GlyphGrabber.HAND_OPEN;
                        handStauts = "open";
                        try {
                            Thread.sleep(1100); // .1 second
                        } catch (InterruptedException ex) {
                            Thread.currentThread().interrupt();
                        }
                        robot.driveTrain.mRight.setPower(-.6);
                        robot.driveTrain.mLeft.setPower(-.6);
                        _state = States.WAIT;
                        try {
                            Thread.sleep(900); // .1 second
                        } catch (InterruptedException ex) {
                            Thread.currentThread().interrupt();
                        }
                        robot.driveTrain.stopHardware();
                        _state = States.STOP;
                    }
                });
                Thread b = new Thread(new Runnable() {
                    @Override
                    public void run()
                    {
                        try {
                            Thread.sleep(3000); // .1 second
                        } catch (InterruptedException ex) {
                            Thread.currentThread().interrupt();
                        }
                        robot.driveTrain.leftPower = .14;
                        robot.driveTrain.rightPower = .14;
                        //robot.jewelRejector.jewelRejectorPosition = .95;
                        try {
                            Thread.sleep(800); // .1 second
                        } catch (InterruptedException ex) {
                            Thread.currentThread().interrupt();
                        }
                        robot.driveTrain.leftPower = 0;
                        robot.driveTrain.rightPower = 0;
                        //robot.jewelRejector.jewelRejectorPosition = .9;
                        try {
                            Thread.sleep(500); // .1 second
                        } catch (InterruptedException ex) {
                            Thread.currentThread().interrupt();
                        }
                        if (_state == States.READING_VALUES) {
                            if (ifFull) {
                                _state = States.MOVE_OFF_PLATE;
                            }
                        }

                        jewelStatus = "skipped";
                    }
                });
                a.start();
                b.start();
                _state = States.READING_VALUES;
                break;
            case READING_VALUES:
                robot.jewelRejector.jewelRejectorPosition = JewelRejector.JEWEL_REJECTOR_DOWN;
                if(useVuforia.run() && readColor.readColor()) {
                    _state = States.HIT_JEWEL;
                }
                break;
            case WAIT_TO_READ:
                try {
                    Thread.sleep(500); // .1 second
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                }
                _state = States.HIT_JEWEL;
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
                robot.driveTrain.leftPower = .25;
                robot.driveTrain.rightPower = .25;
                Thread s = new Thread(new Runnable() {
                    @Override
                    public void run()
                    {
                        double distance;
                        if(ifSide){
                            distance = 24.4;
                        } else {
                            distance = 22.5;
                        }
                        /*while (range.getDistance(DistanceUnit.INCH)  < distance) {
                            robot.driveTrain.leftPower = .25;
                            robot.driveTrain.rightPower = .25;
                        }*/
                        while (robot.driveTrain.mRight.getCurrentPosition() < distance*COUNTS_PER_INCH) {
                            robot.driveTrain.leftPower = .23;
                            robot.driveTrain.rightPower = .23;
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
                            angle = 96;
                        }else{
                            angle = 95;
                        }
                        if (ifBlue) {
                            robot.driveTrain.leftPower = -.25;
                            robot.driveTrain.rightPower = .25;
                            while (useIMU.getHeading() < angle) {
                                robot.driveTrain.leftPower = -.25;
                                robot.driveTrain.rightPower = .25;
                            }
                        } else {
                            robot.driveTrain.leftPower = .25;
                            robot.driveTrain.rightPower = -.25;
                            while (useIMU.getHeading() > -angle) {
                                robot.driveTrain.leftPower = .25;
                                robot.driveTrain.rightPower = -.25;
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
                double leftPower;
                double rightPower;
                if (ifSide) {
                    leftPower = .25;
                    rightPower = .25;
                } else {
                    leftPower = .67;
                    rightPower = 1;

                }
                if (ifFull) {
                    robot.driveTrain.leftPower = leftPower;
                    robot.driveTrain.rightPower = rightPower;
                } else {
                    robot.driveTrain.leftPower = rightPower;
                    robot.driveTrain.rightPower = leftPower;
                }
                Thread v = new Thread(new Runnable() {
                    @Override
                    public void run()
                    {
                        double distance;
                        if(ifSide){
                            distance = 24;
                        }else{
                            distance = 16;
                        }
                        while (true) {
                            if (robot.driveTrain.mRight.getCurrentPosition() > (distance)*COUNTS_PER_INCH)
                                break;
                        }
                        double rightpower;
                        double leftpower;
                        if(ifSide){
                            rightpower = 0.3;
                            leftpower = 0.3;
                        }else{
                            rightpower= 1;
                            leftpower = 0.35;
                        }

                        if (ifBlue) {
                            robot.driveTrain.mLeft.setPower(leftpower);
                            robot.driveTrain.mRight.setPower(rightpower);
                            robot.driveTrain.leftPower = leftpower;
                            robot.driveTrain.rightPower = rightpower;
                        } else {
                            double temp;
                            temp = rightpower;
                            rightpower = leftpower;
                            leftpower = temp;
                            robot.driveTrain.mLeft.setPower(leftpower);
                            robot.driveTrain.mRight.setPower(rightpower);
                            robot.driveTrain.leftPower = leftpower;
                            robot.driveTrain.rightPower = rightpower;
                        }
                        //double distance;
                        if(ifSide){
                            distance = 24;
                        }else {
                            distance = 49;
                        }
                        while (true) {
                            if (robot.driveTrain.mRight.getCurrentPosition() > (distance)*COUNTS_PER_INCH)
                                break;
                        }
                        robot.driveTrain.stopHardware();

                        _state = States.ROTATE_TO_BOX;
                    }
                });
                v.start();

                _state = States.WAIT;
                break;
            case ROTATE_TO_BOX:
                Thread u = new Thread(new Runnable() {
                    @Override
                    public void run()
                    {
                        if (ifSide) {
                            if (ifBlue) {
                                robot.driveTrain.leftPower = .22;
                                robot.driveTrain.rightPower = -.22;
                                while (true) {
                                    if (useIMU.getHeading() <= 103)
                                        break;
                                }
                            } else {
                                robot.driveTrain.leftPower = -.22;
                                robot.driveTrain.rightPower = .22;
                                while (true) {
                                    if (useIMU.getHeading() >= -103)
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
                    distance = 5;
                } else {
                    distance = 20;
                }
                if (robot.driveTrain.mRight.getCurrentPosition() > (distance)*COUNTS_PER_INCH) {
                    _state = States.STOP;
                }
                break;
            case STOP:
                robot.driveTrain.stopHardware();

            default:
                telemetry.addData("Test", "Cry");
        }



        telemetry.addData("State",_state);
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
