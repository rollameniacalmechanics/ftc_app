package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.RelicRecovery.JewelRejector;
import org.firstinspires.ftc.teamcode.Utilities.ReadColor;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;
import org.firstinspires.ftc.teamcode.Utilities.UseIMU;
import org.firstinspires.ftc.teamcode.Utilities.UseVuforia;

/**
 * Created by spmce on 2/1/2018.
 */

@Autonomous(name = "NEW BLUE",group = "aaa")
public class BlueAuto extends OpMode {

    private BigBertha robot;

    ReadColor readColor;
    SetRobot setRobot;
    UseIMU useIMU;
    //UseVuforia useVuforia;

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

    @Override
    public void init() {
        robot = new BigBertha(hardwareMap,telemetry);
        robot.init();
        robot.glyphGrabber.crHand.setPower(-1);

        robot.driveTrain.mLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain.mRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.driveTrain.mLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveTrain.mRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        readColor = new ReadColor(robot.jewelRejector.sColor);
        setRobot = new SetRobot(telemetry);
        useIMU = new UseIMU(hardwareMap, telemetry);
        useIMU.init();
        /*Thread a = new Thread(new Runnable() {
            @Override
            public void run() {
                useVuforia = new UseVuforia(hardwareMap, telemetry);
                useVuforia.init();
            }
        });
        a.run();*/
    }

    public void start() {
        //useVuforia.start();
        useIMU.start();
    }

    private BlueAuto.States _state = States.DOWN;
    private ReadColor.Color jewelColor = ReadColor.Color.NEITHER;

    @Override
    public void loop() {
        robot.run();
        useIMU.run();
        switch (_state) {
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
                if(readColor.readColor()) {
                    _state = States.HIT_JEWEL;
                }
                break;
            case HIT_JEWEL:
                jewelColor = readColor.getColorDetected();
                if (jewelColor == ReadColor.Color.RED) {
                    robot.jewelRejector.jewelRotatorPosition = JewelRejector.JEWEL_ROTATOR_RIGHT;
                } else if (jewelColor == ReadColor.Color.BLUE) {
                    robot.jewelRejector.jewelRotatorPosition = JewelRejector.JEWEL_ROTATOR_LEFT;
                } else {
                    telemetry.addData("Color", "neither, you messed up");
                }
                _state = States.WAIT_TO_MOVE;
                break;
            case WAIT_TO_MOVE:
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
                        double distance = 33.5;
                        while (robot.jewelRejector.sRange.getDistance(DistanceUnit.INCH)  <= distance) {
                            robot.driveTrain.rightPower = 1;
                            robot.driveTrain.leftPower = 1;
                        }
                        robot.driveTrain.stopHardware();

                        _state = States.ROTATE;
                    }
                });
                s.start();
                _state = States.WAIT;
                break;
        }
    }
}
