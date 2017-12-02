package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.RelicRecovery.JewelRejector;
import org.firstinspires.ftc.teamcode.Utilities.ReadColor;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;
import org.firstinspires.ftc.teamcode.Utilities.UseIMU;
import org.firstinspires.ftc.teamcode.Utilities.UseVuforia;


/**
 * Created by Shane on 7/19/2017.
 */
@Autonomous(name = "Backup Relic Recovery Auto Simple", group = "Autonomous")
public class BigBerthaAutoSimple extends OpMode {

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

    ReadColor readColor;
    SetRobot setRobot;

    private boolean ifBlue;
    private int state = 0;
    private boolean part1Done = false;
    private boolean part2Done = false;

    @Override
    public void init() {
        Thread t = new Thread(new Runnable() {
            @Override
            public void run()
            {
                robot = new BigBertha(hardwareMap,telemetry);
                robot.init();
                robot.glyphGrabber.crHand.setPower(-1);

                robot.driveTrain.mLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.driveTrain.mRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.driveTrain.mLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.driveTrain.mRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                readColor = new ReadColor(robot.jewelRejector.sColor);
                telemetry.addData("Robot Init","done");
                part1Done = true;
            }
        });
        Thread u = new Thread(new Runnable() {
            @Override
            public void run()
            {
                setRobot = new SetRobot(telemetry);
                telemetry.addData("Set Robot Init","done");
                part2Done = true;
            }
        });
        t.start();
        u.start();
    }


    enum States {
        READING_VALUES ,
        HIT_JEWEL,
        STOP
    }

    private States _state = States.READING_VALUES;
    private ReadColor.Color jewelColor = ReadColor.Color.NEITHER;

    @Override
    public void loop() {
        robot.run();
        switch(_state) {
            case READING_VALUES:
                robot.jewelRejector.jewelRejectorPosition = JewelRejector.JEWEL_REJECTOR_DOWN;
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
                _state = States.STOP;
                break;
            case STOP:
                robot.driveTrain.leftPower = 0;
                robot.driveTrain.rightPower = 0;
            default:
                telemetry.addData("Test", "Cry");
        }

        telemetry.addData("State",_state);
        telemetry.addData("blue",robot.jewelRejector.sColor.blue());
        telemetry.addData("red",robot.jewelRejector.sColor.red());
        telemetry.addData("right encoder",robot.driveTrain.mRight.getCurrentPosition());
        telemetry.addData("left encoder",robot.driveTrain.mLeft.getCurrentPosition());
        telemetry.addData("arm lift encoder",robot.relicRetriever.mArmLift.getCurrentPosition());
        telemetry.addData("lift encoder",robot.glyphGrabber.mLift.getCurrentPosition());
    }

}
