package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by spmce on 12/1/2017.
 */
@Autonomous(name = "No Thread Relic Recovery Auto Blue Side", group = "No Thread Auto Side")
public class AutoBlueSideNoThread extends OpMode {
    BigBerthaAutoNoThread bigBerthaAuto;
    @Override
    public void init() {
        bigBerthaAuto = new BigBerthaAutoNoThread(hardwareMap,telemetry);
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
        bigBerthaAuto.loop(true,true,true);
    }

    @Override
    public void stop() {
        super.stop();
        bigBerthaAuto.stop();
    }
}
