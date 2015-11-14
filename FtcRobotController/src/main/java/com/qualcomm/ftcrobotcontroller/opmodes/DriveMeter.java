package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

public class DriveMeter extends LinearOpMode {

    Autonomous A;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor L = hardwareMap.dcMotor.get("L");
        DcMotor R = hardwareMap.dcMotor.get("R");
        A = new Autonomous(12.7, 38.1, 1675, L, R);
        A.addSensor("USS1", hardwareMap.ultrasonicSensor.get("USS1"));

        waitForStart();

        while(true) {
            telemetry.addData("USS1: ", A.getSensor("USS1").getUltrasonicLevel());
        }

        //starts at red square and drives to red side ramp
        /*A.DriveDist(230, 0.25);
        A.TurnDegrees(42.5, 0.1);
        A.DriveDist(130,0.4);
        A.stopRobot();*/
    }
}

