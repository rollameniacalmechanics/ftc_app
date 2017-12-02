package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by spmce on 12/1/2017.
 */
@Autonomous(name = "Relic Recovery Auto Red", group = "Auto")
public class AutoRed extends OpMode {
    BigBerthaAuto bigBerthaAuto;
    @Override
    public void init() {
        bigBerthaAuto = new BigBerthaAuto(hardwareMap,telemetry);
        bigBerthaAuto.initAuto(true);
    }
    @Override
    public void init_loop() {
        super.init_loop();
        bigBerthaAuto.init_loop(true);
    }
    @Override
    public void start() {
        super.start();
        bigBerthaAuto.start();
    }

    @Override
    public void loop() {
        bigBerthaAuto.loop(true,false);
    }

    @Override
    public void stop() {
        super.stop();
        bigBerthaAuto.stop();
    }
}
