package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by mharsch on 9/12/15.
 */
public class VSpeedTeleOp extends OpMode {

    DcMotor lMotor;
    DcMotor rMotor;

    boolean wasJustPressedR = false;
    boolean wasJustPressedL = false;
    double slowFactor = 2;
    final double maxBound = 15;
    final double minBound = 1;

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
        rMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double lMotorP = -gamepad1.left_stick_y;
        double rMotorP = -gamepad1.right_stick_y;
        lMotorP /= slowFactor;
        rMotorP /= slowFactor;
        lMotorP = Range.clip(lMotorP, -1, 1);
        rMotorP = Range.clip(rMotorP, -1, 1);
        lMotor.setPower(lMotorP);
        rMotor.setPower(rMotorP);

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
        telemetry.addData("Scaling: ", slowFactor);
        telemetry.addData("Left Motor: ", lMotorP);
        telemetry.addData("Right Motor: ", rMotorP);

    } //end of public void loop
} //end of public class