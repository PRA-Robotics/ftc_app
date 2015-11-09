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
        A = new Autonomous(8.1, 38.1, 1675, L, R);

        waitForStart();

        A.DriveDist(100, 0.25);//Encoder moving to fast could cause problems.
        A.waitForPos();
        A.TurnDegrees(90, 0.1);
        A.waitForPos();
        telemetry.addData("Status: ", "Done");
    }
}

