package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoDrive extends LinearOpMode {

    Autonomous A;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor L = hardwareMap.dcMotor.get("L");
        DcMotor R = hardwareMap.dcMotor.get("R");
        Servo hookServo = hardwareMap.servo.get("H");
        hookServo.setPosition(0.25);
        DcMotor CDM = hardwareMap.dcMotor.get("CDM");
        Servo balServo = hardwareMap.servo.get("BS");
        A = new Autonomous(8.5, 38.2, 1120, L, R);

        waitForStart();
        A.DriveDist(60, 0.25);
        A.TurnDegrees(45, 0.25);
        A.DriveDist(210, 0.25);
        A.TurnDegrees(45, 0.25);
        A.DriveDist(40, 0.25);
        A.waitForPos();

        CDM.setPower(1);
        sleep(2500);
        CDM.setPower(0);

        balServo.setPosition(0.6);

        A.stopRobot();
    }
}
