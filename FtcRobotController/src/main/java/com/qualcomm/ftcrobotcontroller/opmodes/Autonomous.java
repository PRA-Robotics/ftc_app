package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 *  This class is a general purpose utility library for writing autonomous programs.
 *
 *  CAUTION: As of 2015.10.25, this code is untested! This comment will be revised
 *  after testing has been completed.
 */
public class Autonomous{
    private double Diam;
    private double WB;
    private double TPR;
    public DcMotor ML;
    public DcMotor MR;
    public int LT;
    public int RT;
    public int State = 0;

    public Autonomous (double wheelDiameter, double wheelBase, double ticksPerRotation, DcMotor leftMotor, DcMotor rightMotor){
        Diam = wheelDiameter;
        WB = wheelBase;
        TPR = ticksPerRotation;
        ML = leftMotor;
        MR = rightMotor;
        ML.setDirection(DcMotor.Direction.REVERSE);
        ML.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        MR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    public void resetMotor(DcMotor M){
        M.setPower(0);
        M.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    public boolean isDone(DcMotor M, int T){
        int current = M.getCurrentPosition();
        int diff = Math.abs(T-current);
        if(diff <= 10) {
            return (true);
        }
        else {
            return (false);
        }
    }

    public void waitForPos(){
        boolean DL = isDone(ML,LT);
        boolean DR = isDone(MR,RT);
        while(!DL || !DR) {
            if(DL){
                resetMotor(ML);
            }
            if(DR){
                resetMotor(MR);
            }
        }
        resetMotor(ML);
        resetMotor(MR);
    }

    public void DriveDist (double distance, double speed) {
        double cir = Diam * Math.PI;
        double rots = distance/cir;
        double ticks = speed > 0 ? rots * TPR : -rots * TPR;
        LT += (int) ticks;
        RT += (int) ticks;
        ML.setPower(speed);
        MR.setPower(speed);
    }

    public void TurnDegrees (double degrees, double speed){
        double cir = Diam * Math.PI;
        double tCir = WB * Math.PI;
        double dist = tCir * (degrees/360);
        double rots = dist/cir;
        double ticks = speed > 0 ? rots * TPR : -rots * TPR;
        LT += (int) ticks;
        RT -= (int) ticks;
        ML.setPower(speed);
        MR.setPower(-speed);
    }
}
