package org.firstinspires.ftc.teamcode.VelocityVortex;

/**
 * Created by team on 7/19/2017.
 */

public class MyTestCodeTelemetry extends MyTestCodeHardware {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();
    }

    protected void myTelemetry() {
        telemetry.addData("Right Motor Power", rightPower);
        telemetry.addData("Left Motor Power", leftPower);
    }
}
