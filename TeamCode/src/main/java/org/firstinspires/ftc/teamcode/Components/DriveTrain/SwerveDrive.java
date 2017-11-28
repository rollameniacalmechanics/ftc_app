package org.firstinspires.ftc.teamcode.Components.DriveTrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utilities.Map;
import org.firstinspires.ftc.teamcode.Utilities.SetRobot;

/**
 * Created by spmce on 11/26/2017.
 *
 * This drive train is used for robots with a four motor drive train
 */
public class SwerveDrive extends FourMotor {
    // ---------------------- Hardware Devices ----------------------
    /**
     * motor that rotates left wheel
     */
    public DcMotor mSwerveLeft;
    /**
     * motor that rotates right wheel
     */
    public DcMotor mSwerveRight;
    /**
     * motor that rotates back left wheel
     */
    public DcMotor mSwerveBackLeft;
    /**
     * motor that rotates back right wheel
     */
    public DcMotor mSwerveBackRight;
    // --------------------- Hardware Variables ---------------------
    /**
     * this variable is used to set power the swerve left motor
     */
    public double swerveLeftPower;
    /**
     * this variable is used to set power the swerve right motor
     */
    public double swerveRightPower;
    /**
     * this variable is used to set power the swerve back left motor
     */
    public double swerveBackLeftPower;
    /**
     * this variable is used to set power the swerve back right motor
     */
    public double swerveBackRightPower;
    // ------------------------ Constructor -------------------------
    /**
     * Constructs a four motor drive train
     *
     * @param map object that is used to map the drive train
     * @param setRobot object that is used to set the power to the drive train
     */
    public SwerveDrive(Map map, SetRobot setRobot) {
        super(map,setRobot);
        this.map = map;
        this.setRobot = setRobot;
        mSwerveLeft      = null;
        mSwerveRight     = null;
        mSwerveBackLeft  = null;
        mSwerveBackRight = null;
        swerveLeftPower      = 0;
        swerveRightPower     = 0;
        swerveBackLeftPower  = 0;
        swerveBackRightPower = 0;
    }
    // -------------------------- Mapping ---------------------------
    /**
     * Maps left and right motors of drive train
     */
    @Override
    public void initHardware() {
        super.initHardware();
        mSwerveLeft  = map.revMotor("sl");
        mSwerveRight = map.motor("sr");
        mSwerveBackLeft  = map.revMotor("sbl");
        mSwerveBackRight = map.motor("sbr");
    }
    // --------------------- Set Hardware Power ---------------------
    /**
     * Sets power to left and right motors of drive train
     */
    @Override
    public void runHardware() {
        super.runHardware();
        setRobot.power(mSwerveLeft,swerveLeftPower,"swerve left motor");
        setRobot.power(mSwerveRight,swerveRightPower,"swerve right motor");
        setRobot.power(mSwerveBackLeft,swerveBackLeftPower,"swerve back left motor");
        setRobot.power(mSwerveBackRight,swerveBackRightPower,"swerve back right motor");
    }
}
