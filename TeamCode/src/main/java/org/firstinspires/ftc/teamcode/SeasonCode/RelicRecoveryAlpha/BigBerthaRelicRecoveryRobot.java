package org.firstinspires.ftc.teamcode.SeasonCode.RelicRecoveryAlpha;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.GlyphGrabber.GlyphGrabberHardware;
import org.firstinspires.ftc.teamcode.Components.GlyphGrabber.Grabber;
import org.firstinspires.ftc.teamcode.Components.JewelRejector.JewelRejector;
import org.firstinspires.ftc.teamcode.Components.RelicRetriever.RelicRetrieverHardware;
import org.firstinspires.ftc.teamcode.Components.RelicRetriever.Retriever;
import org.firstinspires.ftc.teamcode.Robot.Robot;

/**
 * Created by Shane on 26-11-2017.
 */
public class BigBerthaRelicRecoveryRobot extends Robot {
    GlyphGrabberHardware glyphGrabber;
    RelicRetrieverHardware relicRetriever;
    JewelRejector jewelRejector;
    // ------------------------ Constructor -------------------------
    BigBerthaRelicRecoveryRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap,telemetry);
        glyphGrabber = new Grabber(map,setRobot);
        relicRetriever = new Retriever(map,setRobot);
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
