package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class TeleOp extends OpMode {

    DcMotor lMotor;
    DcMotor rMotor;
    DcMotor CDM;
    Servo balServo;
    Servo hookServo;

    boolean wasJustPressedR = false;
    boolean wasJustPressedL = false;
    boolean driveMode = true;
    double slowFactor = 3;
    double railPosition;
    double balPosition;
    final double maxBound = 15;
    final double minBound = 2;

    public double normalize(double val) {
        if (val > maxBound) {
            val = maxBound;
         }
        if (val < minBound) {
            val = minBound;
        }
        return val;
    }

    @Override
     public void init() {
        lMotor = hardwareMap.dcMotor.get("L");
        rMotor = hardwareMap.dcMotor.get("R");
        hookServo = hardwareMap.servo.get("H");
        CDM = hardwareMap.dcMotor.get("CDM");
        balServo = hardwareMap.servo.get("BS");
        rMotor.setDirection(DcMotor.Direction.REVERSE);
        railPosition = 0.25;
        balPosition = 1.0;
    }

    /*
     * Switch controller modes: Start Button
     *
     * Controller Mode 1:
     *     Left motor: Left Joystick-Y
     *     Right motor: Right Joystick-Y
     *     Speed Control: L-Bumper & R-Bumper
     *     Hook Control: A & B
     *     Tower Expand/Contract: D-Pad Up/Down
     *     Balance Tower: D-Pad Right/Left
     * Controller Mode 2:
     *     Extend Crane Arm: Left Joystick-Y
     *     Rotate Crane Arm: Right Joystick-X
     *     Raise/Lower Claw: D-Pad Up/Down
     *     Open/Close Claw: L-Trigger & R-Trigger
     */

    @Override
    public void loop() {
        if (driveMode) {
            double lMotorP = -gamepad1.left_stick_y;
            double rMotorP = -gamepad1.right_stick_y;
            lMotorP /= slowFactor;
            rMotorP /= slowFactor;
            lMotorP = Range.clip(lMotorP, -1, 1);
            rMotorP = Range.clip(rMotorP, -1, 1);
            railPosition = Range.clip(railPosition, 0.3, 1);
            hookServo.setPosition(railPosition);
            balPosition = Range.clip(balPosition, 0.25, 1);
            balServo.setPosition(balPosition);
            telemetry.addData("Speed: ", slowFactor);
            lMotor.setPower(lMotorP);
            rMotor.setPower(rMotorP);

            if (gamepad1.dpad_up) {
                CDM.setPower(1.0);
            }
            else if (gamepad1.dpad_down) {
                CDM.setPower(-1.0);
            }
            else {
                CDM.setPower(0);
            }
            if (gamepad1.dpad_left) {
                balPosition += 0.001;
            }
            if (gamepad1.dpad_right) {
                balPosition -= 0.001;
            }
            if(gamepad1.a){
                    railPosition += 0.001;
            }
            if(gamepad1.y){
                    railPosition -= 0.001;
            }
            if (gamepad1.right_bumper && !wasJustPressedR) {
                wasJustPressedR = true;
                slowFactor += 1;
                slowFactor = normalize(slowFactor);
            } //end of if statement
            if (!gamepad1.right_bumper) {
                wasJustPressedR = false;
            } //end of if statement
            if (gamepad1.left_bumper && !wasJustPressedL) {
                wasJustPressedL = true;
                slowFactor -= 1;
                slowFactor = normalize(slowFactor);
            } //end of if statement
            if (!gamepad1.left_bumper) {
                wasJustPressedL = false;
            } //end of if statement
        }

    } //end of public void loop
} //end of public class