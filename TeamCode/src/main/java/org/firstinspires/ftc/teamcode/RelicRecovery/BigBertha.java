package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.RelicRecovery.GlyphGrabber;
import org.firstinspires.ftc.teamcode.Components.RelicRecovery.JewelRejector;
import org.firstinspires.ftc.teamcode.Components.RelicRecovery.RelicRetriever;
import org.firstinspires.ftc.teamcode.Robot.Robot;

/**
 * Created by Shane on 26-11-2017.
 */
public class BigBertha extends Robot {
    // ----------------------- Robot Objects ------------------------

    /**
     * addd hereeeee
     */
    GlyphGrabber glyphGrabber;
    RelicRetriever relicRetriever;
    JewelRejector jewelRejector;
    // ------------------------ Constructor -------------------------
    BigBertha(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap,telemetry);
        glyphGrabber = new GlyphGrabber(map,setRobot);
        relicRetriever = new RelicRetriever(map,setRobot);
        jewelRejector = new JewelRejector(map,setRobot);
    }
    // -------------------------- Mapping ---------------------------
    public void mapHardware() {
        relicRetriever.initHardware();
        glyphGrabber.initHardware();
        jewelRejector.initHardware();
    }
    // --------------------- Set Hardware Power ---------------------
    public void setHardwarePowers() {
        glyphGrabber.runHardware();
        relicRetriever.runHardware();
        jewelRejector.runHardware();
    }
}
