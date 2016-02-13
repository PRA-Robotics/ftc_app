package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class TeleOp extends OpMode {

    DcMotor lMotor;
    DcMotor rMotor;
    DcMotor CDM;
    Servo balServo;
    Servo hookServo;
    Servo rotServo;
    Servo skwServo;
    Servo winchServo;
    Servo clawServo;

    boolean wasJustPressedR = false;
    boolean wasJustPressedL = false;
    boolean wasJustPressedBack = false;
    boolean pMode = false;
    double slowFactor = 3;
    double hookPosition;
    double balPosition;
    double rotPosition;
    double skwPosition;
    double winchPosition;
    double clawPosition;
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
        CDM = hardwareMap.dcMotor.get("CDM");
        hookServo = hardwareMap.servo.get("H");
        balServo = hardwareMap.servo.get("BS");
        rotServo = hardwareMap.servo.get("ROT");
        skwServo = hardwareMap.servo.get("SKW");
        winchServo = hardwareMap.servo.get("LFT");
        clawServo = hardwareMap.servo.get("CLW");
        rMotor.setDirection(DcMotor.Direction.REVERSE);
        hookPosition = 0.25;
        balPosition = 1.0;
        rotPosition = 0;
        skwPosition = 0;
        winchPosition = 0.5;
        clawPosition = 1;
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
     *     Open/Close Claw: L-Bumper & R-Bumper
     *     Precise Mode: Back Button
     */

    //Add functionality for 2 controllers. Allow each controller to be in its own mode. One could
    //control the crane and the other driving, or they could both control the same mode.

    public void craneMode(Gamepad controller) {
        telemetry.addData("Driver " + controller.user + " mode: ", "CraneMode");
        hookPosition = Range.clip(hookPosition, 0.3, 0.83);
        balPosition = Range.clip(balPosition, 0.25, 1);
        rotPosition = Range.clip(rotPosition, 0, 0.53);
        clawPosition = Range.clip(clawPosition, 0.25, 1);
        skwPosition = Range.clip(skwPosition, 0, 0.575);
        hookServo.setPosition(hookPosition);
        balServo.setPosition(balPosition);
        rotServo.setPosition(rotPosition);
        clawServo.setPosition(clawPosition);
        skwServo.setPosition(skwPosition);
        winchServo.setPosition(winchPosition);

        if(controller.back && ! wasJustPressedBack) {
            pMode = !pMode;
            wasJustPressedBack = true;
        }
        if(!controller.back) {
            wasJustPressedBack = false;
        }

        if(-controller.left_stick_y > 0.1) {
            skwPosition += 0.001;
        }
        if(-controller.left_stick_y < -0.1) {
            skwPosition -= 0.001;
        }

        if(controller.right_stick_x > 0.1) {
            rotPosition -= 0.00025;
        }
        if(controller.right_stick_x < -0.1) {
            rotPosition += 0.00025;
        }

        if(controller.dpad_up) {
            winchPosition = 0;
        }
        else if(controller.dpad_down) {
            winchPosition = 1;
        }
        else {
            winchPosition = 0.5;
        }

        if(controller.left_bumper) {
            clawPosition = 1;
        }
        if(controller.right_bumper) {
            clawPosition = 0;
        }

    }

    public void driveMode(Gamepad controller) {
        telemetry.addData("Driver " + controller.user + " mode: ", "DriveMode");
        double lMotorP = -controller.left_stick_y;
        double rMotorP = -controller.right_stick_y;
        lMotorP /= slowFactor;
        rMotorP /= slowFactor;
        lMotorP = Range.clip(lMotorP, -1, 1);
        rMotorP = Range.clip(rMotorP, -1, 1);
        hookPosition = Range.clip(hookPosition, 0.3, 0.83);
        balPosition = Range.clip(balPosition, 0.25, 1);
        hookServo.setPosition(hookPosition);
        balServo.setPosition(balPosition);
        winchServo.setPosition(winchPosition);
        telemetry.addData("Speed: ", slowFactor);
        lMotor.setPower(lMotorP);
        rMotor.setPower(rMotorP);

        if (controller.dpad_up) {
            CDM.setPower(1.0);
        }
        else if (controller.dpad_down) {
            CDM.setPower(-1.0);
        }
        else {
            CDM.setPower(0);
        }
        if (controller.dpad_left) {
            balPosition += 0.001;
        }
        if (controller.dpad_right) {
            balPosition -= 0.001;
        }
        if(controller.a){
            hookPosition += 0.001;
        }
        if(controller.y){
            hookPosition -= 0.001;
        }
        if (controller.right_bumper && !wasJustPressedR) {
            wasJustPressedR = true;
            slowFactor += 1;
            slowFactor = normalize(slowFactor);
        }
        if (!controller.right_bumper) {
            wasJustPressedR = false;
        }
        if (controller.left_bumper && !wasJustPressedL) {
            wasJustPressedL = true;
            slowFactor -= 1;
            slowFactor = normalize(slowFactor);
        }
        if (!controller.left_bumper) {
            wasJustPressedL = false;
        }

    }

    @Override
    public void loop() {
        driveMode(gamepad1);
        craneMode(gamepad2);

    } //end of public void loop
} //end of public class