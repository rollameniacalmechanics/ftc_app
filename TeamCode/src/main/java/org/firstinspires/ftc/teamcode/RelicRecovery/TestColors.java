package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Utilities.ReadColor;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;

/**
 * Created by lsatt on 11/25/2017.
 *
 * Class to Test Color Sensor
 */

@Autonomous(name = "Relic Recovery Test", group = "Autonomous")
public class TestColors extends OpMode {
    BigBertha robot;
    ReadColor readColor;
    SetRobot setRobot;
    @Override
    public void init() {
        robot = new BigBertha(hardwareMap,telemetry);
        readColor = new ReadColor(robot.jewelRejector.sColor);
        setRobot = new SetRobot(telemetry);
    }

    @Override
    public void loop() {
        robot.jewelRejector.jewelRejectorPosition = robot.jewelRejector.JEWEL_REJECTOR_DOWN;
        setRobot.position(robot.jewelRejector.ssBallPusher,robot.jewelRejector.jewelRejectorPosition,"ball pusher");
        telemetry.addData("blue",robot.jewelRejector.sColor.blue());
        telemetry.addData("red",robot.jewelRejector.sColor.red());
    }
}
