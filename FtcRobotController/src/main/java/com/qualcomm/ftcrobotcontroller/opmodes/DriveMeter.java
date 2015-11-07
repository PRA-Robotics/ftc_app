package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveMeter extends LinearOpMode {

    Autonomous A;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor L = hardwareMap.dcMotor.get("L");
        DcMotor R = hardwareMap.dcMotor.get("R");
        A = new Autonomous(12.7, 38.1, 1675, L, R);

        waitForStart();

        A.TurnDegrees(100, 0.25);//Encoder moving to fast could cause problems.
        A.waitForPos();
        telemetry.addData("Status: ", "Done");
    }
}

