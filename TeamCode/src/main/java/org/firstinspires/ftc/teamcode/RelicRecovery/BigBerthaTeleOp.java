package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.RelicRecovery.GlyphGrabber;
import org.firstinspires.ftc.teamcode.Components.RelicRecovery.JewelRejector;
import org.firstinspires.ftc.teamcode.Components.RelicRecovery.RelicRetriever;
import org.firstinspires.ftc.teamcode.Methods.DriveTrainMethods.TankDriveMethods;
import org.firstinspires.ftc.teamcode.Methods.DriveTrainMethods.TurnDriveMethods;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Shane on 26-11-2017.
 *
 * Alpha Code for 6168
 */
@TeleOp(name = "Relic Recovery TeleOp",group = "TeleOp")
public class BigBerthaTeleOp extends OpMode {
    // --------------------- Private Variables ----------------------
    private BigBertha robot;
    private String driveMode;
    private int driveConfig = 0;
    private boolean ifHold;
    private boolean slowDrive = false;
    private boolean leftStick1 = true;
    private boolean rightStick1 = true;
    private boolean defaultDrive = true;
    private boolean inverseDrive = false;

    private boolean isPad1dYPressed;
    private boolean isPad1dYReleased;

    private boolean isPad1XPressed;
    private boolean isPad1XReleased;
    private boolean isPad1YPressed;
    private boolean isPad1YReleased;
    private boolean isPad2YPressed;
    private boolean isPad2YReleased;



    // ----------------------- Public Methods -----------------------
    // ----------------------- Init -----------------------
    @Override
    public void init() {
        robot = new BigBertha(hardwareMap,telemetry);
        robot.init();
        driveMode = "Tank Drive Init";
        ifHold = false;
        tele();
    }
    // ----------------------- Loop -----------------------
    @Override
    public void loop() {
        padControls();
        tele();
        robot.run();
    }
    // ---------------------- Private Methods -----------------------
    // ----------------------- Pads -----------------------
    private void padControls() {
        gamepad1Controls();
        gamepad2Controls();
        if (gamepad1.dpad_right && !defaultDrive || gamepad2.dpad_right) {
            ifHold = true;
        } else if (gamepad1.dpad_left && !defaultDrive || gamepad2.dpad_left) {
            ifHold = false;
            robot.glyphGrabber.crHandPosition = GlyphGrabber.HAND_OPEN;
        }
        if (ifHold) {
            robot.glyphGrabber.crHandPosition = GlyphGrabber.HAND_CLOSED;
        } else {
            if (gamepad1.dpad_left  && !defaultDrive || gamepad2.dpad_left) {
                robot.glyphGrabber.crHandPosition = GlyphGrabber.HAND_OPEN;
            } else {
                robot.glyphGrabber.crHandPosition = GlyphGrabber.HAND_STOPPED;
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
                isPad1dYPressed = true;
                isPad1dYReleased = false;
            }
            if (isPad1dYPressed) {
                if (!gamepad1.y) {
                    isPad1dYPressed = false;
                    isPad1dYReleased = true;
                }
                if (isPad1dYReleased) {
                    driveConfig++;
                    if (driveConfig == 2) {
                        driveConfig = 0;
                    }
                }
            }
            if (driveConfig == 0) {
                TankDriveMethods tank = new TankDriveMethods();
                drivePower = tank.drive(gamepad1);
                driveMode = "Tank Drive";
            }
            if (driveConfig == 1) {
                TurnDriveMethods turn = new TurnDriveMethods();
                drivePower = turn.drive(gamepad1);
                driveMode = "Turn Drive";
            }
            if (gamepad1.dpad_up) {
                inverseDrive = true;
            } else if (gamepad1.dpad_down) {
                inverseDrive = false;
            }
            if (inverseDrive) {
                double temp = robot.driveTrain.rightPower;
                robot.driveTrain.rightPower = -robot.driveTrain.leftPower;
                robot.driveTrain.leftPower = -temp;
            }
            robot.glyphGrabber.liftPower = 0;
            robot.relicRetriever.armLiftPower = 0;
            robot.relicRetriever.armPower = 0;
        } else { // for gamepad 1 controls override
            TankDriveMethods tank = new TankDriveMethods();
            drivePower = tank.drive(gamepad1);
            driveMode = "Tank Drive";
            // ---- relic override ----
            if (gamepad1.y) {
                isPad1YPressed = true;
                isPad1YReleased = false;
            }
            if (isPad1YPressed) {
                if (!gamepad1.y) {
                    isPad1YPressed = false;
                    isPad1YReleased = true;
                }
                if (isPad1YReleased) {
                    if (robot.relicRetriever.grabberPosition == RelicRetriever.GRABBER_OPEN) {
                        robot.relicRetriever.grabberPosition = RelicRetriever.GRABBER_CLOSED;
                    } else {
                        robot.relicRetriever.grabberPosition = RelicRetriever.GRABBER_OPEN;
                    }
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
            /*if (gamepad1.right_bumper) {
                robot.relicRetriever.armPosition += .001;
            } else if (gamepad1.left_bumper) {
                robot.relicRetriever.armPosition -=.001;
            }*/
            if (gamepad1.right_bumper) {
                robot.relicRetriever.armPower = 1;

            } else if (gamepad1.left_bumper){
                robot.relicRetriever.armPower = -1;
            } else {
                robot.relicRetriever.armPower = 0;
            }
            robot.relicRetriever.armLiftPower = gamepad1.right_trigger - gamepad1.left_trigger;
        }
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
            robot.driveTrain.rightPower *= .75;
            robot.driveTrain.leftPower *= .75;
        }
        // --------------- Jewels ---------------
        if (gamepad1.x) {
            isPad1XPressed = true;
            isPad1XReleased = false;
        }
        if (isPad1XPressed) {
            if (!gamepad1.x) {
                isPad1XPressed = false;
                isPad1XReleased = true;
            }
            if (isPad1XReleased) {
                if (robot.jewelRejector.jewelRejectorPosition == JewelRejector.JEWEL_REJECTOR_UP) {
                    robot.jewelRejector.jewelRejectorPosition = JewelRejector.JEWEL_REJECTOR_DOWN;
                } else {
                    robot.jewelRejector.jewelRejectorPosition = JewelRejector.JEWEL_REJECTOR_UP;
                }
            }
        }

        if (gamepad1.b) {
            robot.jewelRejector.jewelRotatorPosition = JewelRejector.JEWEL_ROTATOR_RIGHT;
        } else if (gamepad1.a) {
            robot.jewelRejector.jewelRotatorPosition = JewelRejector.JEWEL_ROTATOR_LEFT;
        } else {
            robot.jewelRejector.jewelRotatorPosition = JewelRejector.JEWEL_ROTATOR_CENTER;
        }
    }
    // ---------------------- Pad 2 -----------------------
    private void gamepad2Controls() {
        // ---------------- Lift ----------------
        robot.glyphGrabber.liftPower -= gamepad2.right_stick_y;
        // -------------- GlyphGrabber ---------------

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
                if (robot.relicRetriever.grabberPosition == RelicRetriever.GRABBER_OPEN) {
                    robot.relicRetriever.grabberPosition = RelicRetriever.GRABBER_CLOSED;
                } else {
                    robot.relicRetriever.grabberPosition = RelicRetriever.GRABBER_OPEN;
                }
            }
        }
        // ---------------- Arm -----------------
        /*if (gamepad2.right_bumper) {
            robot.relicRetriever.armPosition += .0008;
        } else if (gamepad2.left_bumper) {
            robot.relicRetriever.armPosition -=.0008;
            if (robot.relicRetriever.armPosition < 0) {
                robot.relicRetriever.armPosition = 0;
            } else if (robot.relicRetriever.armPosition > 1) {
                    robot.relicRetriever.armPosition = 1;
            }
        }*/
        double relicPower = 1;
        if (gamepad2.x) {
            relicPower = .5;
        }
        if (gamepad2.right_bumper) {
            robot.relicRetriever.armPower = relicPower;
        } else if (gamepad2.left_bumper){
            robot.relicRetriever.armPower = -relicPower;
        } /*else {
            robot.relicRetriever.armPower = 0;
        }*/
        robot.relicRetriever.armLiftPower += gamepad2.right_trigger - gamepad2.left_trigger;
    }

    private void tele() {
        ifDriveOverride();
        driveMode();
        slowDrive();
        ifHold();
        telemetry.addData("distance", robot.jewelRejector.sRange.getDistance(DistanceUnit.INCH));

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
    private void inverseDrive() {
        telemetry.addData("If Inverse Drive", inverseDrive);
    }
    private void ifHold() {
        telemetry.addData("If Relic Held", ifHold);
    }
}

