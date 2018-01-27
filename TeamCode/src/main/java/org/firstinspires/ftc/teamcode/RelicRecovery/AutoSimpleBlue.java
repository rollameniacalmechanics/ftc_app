package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by spmce on 12/1/2017.
 */
@Autonomous(name = "Relic Recovery Auto Simple Blue", group = "Auto")
public class AutoSimpleBlue extends OpMode {
    BigBerthaAuto bigBerthaAuto;
    @Override
    public void init() {
        bigBerthaAuto = new BigBerthaAuto(hardwareMap,telemetry);
        bigBerthaAuto.initAuto(false);
    }
    @Override
    public void init_loop() {
        super.init_loop();
        bigBerthaAuto.init_loop(false);
    }
    @Override
    public void loop() {
        bigBerthaAuto.loop(false,true,false);
    }

}
