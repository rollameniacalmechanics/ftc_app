package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.DriveTrain.FourMotor;
import org.firstinspires.ftc.teamcode.Components.DriveTrain.StandardDrive;
import org.firstinspires.ftc.teamcode.Utilities.Map;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;

/**
 * Created by Shane on 26-11-2017.
 *
 * Robot abstract class
 */
public abstract class Robot implements FTCRobot {
    // ---------------------- Hardware Objects ----------------------
    /**
     * object used to map hardware
     */
    protected Map map;
    /**
     * object used to set component power
     */
    protected SetRobot setRobot;
    /**
     * default drive train
     */
    public StandardDrive driveTrain;
    // ------------------------ Constructor -------------------------
    /**
     * Initializes map and setRobot objects
     * Initializes default drive train
     *
     * @param hardwareMap maps hardware to phones
     * @param telemetry used to output to phones
     */
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        map = new Map(hardwareMap,telemetry);
        setRobot = new SetRobot(telemetry);
        driveTrain = new StandardDrive(map,setRobot);
    }
    // -------------------------- Mapping ---------------------------
    /**
     * Initializes drive train and maps hardware
     */
    public void init() {
        driveTrain.initHardware();
        mapHardware();
    }
    // --------------------- Set Hardware Power ---------------------
    /**
     * Sets power to drive train and other hardware devices
     */
    public void run() {
        driveTrain.runHardware();
        setHardwarePowers();
    }
}
