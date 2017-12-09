package org.firstinspires.ftc.teamcode.Components.DriveTrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utilities.Map;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;

/**
 * Created by spmce on 11/26/2017.
 *
 * This drive train is used for robots with a four motor drive train
 */
public class FourMotor extends StandardDrive {

    // --------------------- Hardware Variables ---------------------
    /**
     * this variable is used to set power the back left motor
     */
    public double backLeftPower;
    /**
     * this variable is used to set power the back right motor
     */
    public double backRightPower;
    // ------------------------ Constructor -------------------------
    /**
     * Constructs a four motor drive train
     *
     * @param map object that is used to map the drive train
     * @param setRobot object that is used to set the power to the drive train
     */
    public FourMotor(Map map, SetRobot setRobot) {
        super(map,setRobot);
        this.map = map;
        this.setRobot = setRobot;
        backLeftPower  = 0;
        backRightPower = 0;
    }
    // -------------------------- Mapping ---------------------------
    /**
     * Maps left and right motors of drive train
     */
    @Override
    public void initHardware() {
        super.initHardware();
    }
    // --------------------- Set Hardware Power ---------------------
    /**
     * Sets power to left and right motors of drive train
     */
    @Override
    public void runHardware() {
        setRobot.power(mLeft,leftPower,"left motor");
        setRobot.power(mRight,rightPower,"right motor");
        setRobot.power(mBackLeft,backLeftPower,"back left motor");
        setRobot.power(mBackRight,backRightPower,"back right motor");
    }
    public void stopHardware() {
        super.stopHardware();
        backLeftPower  = 0;
        backRightPower = 0;
    }
}
