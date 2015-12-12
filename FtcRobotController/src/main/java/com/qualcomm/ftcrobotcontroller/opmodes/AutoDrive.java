package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class AutoDrive extends LinearOpMode {

    Autonomous A;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor L = hardwareMap.dcMotor.get("L");
        DcMotor R = hardwareMap.dcMotor.get("R");
        A = new Autonomous(12.7, 38.2, 1120, L, R);

        waitForStart();

        A.DriveDist(144.5,-0.3);
        A.TurnDegrees(90,0.25);
        A.DriveDist(100,-0.3);
        //drop climbers into bin and hit correct button
        A.TurnDegrees(180,-0.25);
        A.DriveDist(100,-0.3);
        A.TurnDegrees(90,0.25);
        A.DriveDist(123,-0.3);
        A.TurnDegrees(45,0.25);
        A.DriveDist(120,-1);

        A.stopRobot();
    }
}
