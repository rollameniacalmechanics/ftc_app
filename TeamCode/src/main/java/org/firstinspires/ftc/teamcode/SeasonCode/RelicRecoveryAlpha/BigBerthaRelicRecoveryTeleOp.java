package org.firstinspires.ftc.teamcode.SeasonCode.RelicRecoveryAlpha;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Methods.DriveTrainMethods.TankDriveMethods;
import org.firstinspires.ftc.teamcode.Methods.DriveTrainMethods.TurnDriveMethods;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Utilities.ServoPositions.BALL_PUSHER_DOWN;
import static org.firstinspires.ftc.teamcode.Utilities.ServoPositions.BALL_PUSHER_UP;
import static org.firstinspires.ftc.teamcode.Utilities.ServoPositions.BALL_ROTATOR_CENTER;
import static org.firstinspires.ftc.teamcode.Utilities.ServoPositions.BALL_ROTATOR_LEFT;
import static org.firstinspires.ftc.teamcode.Utilities.ServoPositions.BALL_ROTATOR_RIGHT;
import static org.firstinspires.ftc.teamcode.Utilities.ServoPositions.GRABBER_CLOSED;
import static org.firstinspires.ftc.teamcode.Utilities.ServoPositions.GRABBER_OPEN;
import static org.firstinspires.ftc.teamcode.Utilities.ServoPositions.HAND_CLOSED;
import static org.firstinspires.ftc.teamcode.Utilities.ServoPositions.HAND_OPEN;
import static org.firstinspires.ftc.teamcode.Utilities.ServoPositions.HAND_STOPPED;

/**
 * Created by Shane on 26-11-2017.
 */
@TeleOp(name = "Relic Recovery TeleOp Alpha",group = "TeleOp")
public class BigBerthaRelicRecoveryTeleOp extends OpMode {
    // --------------------- Private Variables ----------------------
    private BigBerthaRelicRecoveryRobot robot;
    private String driveMode;
    private int driveConfig = 0;
    private boolean ifHold;
    private boolean RelicHold;
    private boolean slowDrive = false;
    private boolean leftStick1 = true;
    private boolean rightStick1 = true;
    private boolean x1 = true;
    private boolean y1 = true;
    private boolean defaultDrive = true;

    private boolean isPad1XPressed;
    private boolean isPad1XReleased;
    private boolean isPad2YPressed;
    private boolean isPad2YReleased;

    // ----------------------- Public Methods -----------------------
    // ----------------------- Init -----------------------
    @Override
    public void init() {
        robot = new BigBerthaRelicRecoveryRobot(hardwareMap,telemetry);
        robot.init();
        driveMode = "Turn Drive Init";
        ifHold = false;
        tele();
    }
    // ----------------------- Loop -----------------------
    @Override
    public void loop() {
        padControls();
        tele();
        robot.setHardwarePower();
    }
    // ---------------------- Private Methods -----------------------
    // ----------------------- Pads -----------------------
    private void padControls() {
        gamepad1Controls();
        gamepad2Controls();
        if (gamepad1.dpad_right || gamepad2.dpad_right) {
            ifHold = true;
        } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
            ifHold = false;
            robot.glyphGrabber.crHandPosition = HAND_OPEN;
        }
        if (ifHold) {
            robot.glyphGrabber.crHandPosition = HAND_CLOSED;
        } else {
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                robot.glyphGrabber.crHandPosition = HAND_OPEN;
            } else {
                robot.glyphGrabber.crHandPosition = HAND_STOPPED;
            }

        }
    }
    // ---------------------- Pad 1 -----------------------
    private void gamepad1Controls() {
        // ------------ Drive Train -------------
        if (gamepad1.left_stick_button) {
            if (leftStick1) {
                defaultDrive = !defaultDrive;
                leftStick1 = false;
            }
        } else {
            leftStick1 = true;
        }
        double drivePower[] = new double[2];
        if (defaultDrive) {
            if (gamepad1.y) {
                driveConfig++;
                if (driveConfig == 2) {
                    driveConfig = 0;
                }
            }
            if (gamepad1.x) {
                driveConfig--;
                if (driveConfig == -1) {
                    driveConfig = 1;
                }
            }
            if (driveConfig == 0) {
                TurnDriveMethods turn = new TurnDriveMethods();
                drivePower = turn.drive(gamepad1);
                driveMode = "Turn Drive";
            }
            if (driveConfig == 1) {
                TankDriveMethods tank = new TankDriveMethods();
                drivePower = tank.drive(gamepad1);
                driveMode = "Tank Drive";
            }
            robot.glyphGrabber.liftPower = 0;
        } else { // for gamepad 1 controls override
            // ---- relic override ----
            if (gamepad1.y) {
                if (y1) {
                    if (robot.relicRetriever.grabberPosition == GRABBER_OPEN) {
                        robot.relicRetriever.grabberPosition = GRABBER_CLOSED;
                    } else {
                        robot.relicRetriever.grabberPosition = GRABBER_OPEN;
                    }
                    y1 = false;
                } else {
                    y1 = true;
                }
            }
            // ---- lift override -----
            if (gamepad1.dpad_up) {
                robot.glyphGrabber.liftPower = 1;
            } else if (gamepad1.dpad_down) {
                robot.glyphGrabber.liftPower = -1;
            } else {
                robot.glyphGrabber.liftPower = 0;
            }
            // ----- arm override -----
            if (gamepad1.right_bumper) {
                robot.relicRetriever.armPosition += .02;
            } else if (gamepad1.left_bumper) {
                robot.relicRetriever.armPosition -=.02;
            }
            robot.relicRetriever.armLiftPower = gamepad1.right_trigger - gamepad1.left_trigger;
        }
        //robot.rightPower = drivePower[0];
        //robot.leftPower = drivePower[1];
        robot.driveTrain.rightPower = drivePower[0];
        robot.driveTrain.leftPower = drivePower[1];
        if (gamepad1.right_stick_button) {
            if (rightStick1) {
                slowDrive = !slowDrive;
                rightStick1 = false;
            }
        } else {
            rightStick1 = true;
        }
        if (slowDrive) {
            //robot.rightPower /= 2;
            //robot.leftPower /= 2;
            robot.driveTrain.rightPower /= 2;
            robot.driveTrain.leftPower /= 2;
        }
        // --------------- Jewels ---------------
        if (gamepad1.x) {
            isPad1XPressed = true;
            isPad1XReleased = false;
        }
        if (isPad1XPressed) {
            if (!gamepad1.x) {
                isPad1XReleased = true;
                isPad1XPressed = false;
            }
            if (isPad1XReleased) {
                if (robot.jewelRejector.ballPusherPosition == BALL_PUSHER_UP) {
                    robot.jewelRejector.ballPusherPosition = BALL_PUSHER_DOWN;
                } else {
                    robot.jewelRejector.ballPusherPosition = BALL_PUSHER_UP;
                }
            }
        }

        if (gamepad1.b) {
            robot.jewelRejector.ballRotatorPosition = BALL_ROTATOR_RIGHT;
        } else if (gamepad1.a) {
            robot.jewelRejector.ballRotatorPosition = BALL_ROTATOR_LEFT;
        } else {
            robot.jewelRejector.ballRotatorPosition = BALL_ROTATOR_CENTER;
        }
    }
    // ---------------------- Pad 2 -----------------------
    private void gamepad2Controls() {
        // ---------------- Lift ----------------
        robot.glyphGrabber.liftPower -= gamepad2.right_stick_y;
        // -------------- Grabber ---------------

        if (gamepad2.y) {
            isPad2YPressed = true;
            isPad2YReleased = false;
        }
        if (isPad2YPressed) {
            if (!gamepad2.y) {
                isPad2YReleased = true;
                isPad2YPressed = false;
            }
            if (isPad2YReleased) {
                if (robot.relicRetriever.grabberPosition == HAND_OPEN) {
                    robot.relicRetriever.grabberPosition = HAND_CLOSED;
                } else {
                    robot.relicRetriever.grabberPosition = HAND_OPEN;
                }
            }
        }
        // ---------------- Arm -----------------
        if (gamepad2.right_bumper) {
            robot.relicRetriever.armPosition += .0008;
        } else if (gamepad2.left_bumper) {
            robot.relicRetriever.armPosition -=.0008;
            if (robot.relicRetriever.armPosition < 0)
                robot.relicRetriever.armPosition = 0;
        }
        robot.relicRetriever.armLiftPower = gamepad2.right_trigger - gamepad2.left_trigger;
    }

    private void tele() {
        ifDriveOverride();
        driveMode();
        slowDrive();
        ifHold();
    }
    private void ifDriveOverride() {
        telemetry.addData("If default drive", defaultDrive);
    }
    private void driveMode() {
        telemetry.addData("Drive Mode", driveMode);
    }
    private void slowDrive() {
        telemetry.addData("If Slow Drive", slowDrive);
    }
    private void ifHold() {
        telemetry.addData("If Relic Held", ifHold);
    }
}

