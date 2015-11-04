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
    public DcMotor MR;
    public DcMotor ML;
    public int State = 0;

    public Autonomous (double wheelDiameter, double wheelBase, double ticksPerRotation, DcMotor leftMotor, DcMotor rightMotor){
        Diam = wheelDiameter;
        WB = wheelBase;
        TPR = ticksPerRotation;
        ML = leftMotor;
        MR = rightMotor;
        ML.setDirection(DcMotor.Direction.REVERSE);
        ML.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        MR.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
    }

    public void resetMotors(){
        ML.setPower(0);
        MR.setPower(0);
        ML.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        MR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    public boolean isDone(DcMotor M){
        int target = M.getTargetPosition();
        int current = M.getCurrentPosition();
        int diff = Math.abs(target-current);
        if(diff <= 10) {
            return true;
        }
        else {
            return false;
        }
    }

    public void DriveDist (double distance, double speed) {
        double cir = Diam * Math.PI;
        double rots = distance/cir;
        double ticks = speed > 0 ? rots * TPR : -rots * TPR;
        ML.setTargetPosition((int) ticks);
        MR.setTargetPosition((int) ticks);
        ML.setPower(speed);
        MR.setPower(speed);
    }

    public void TurnDegrees (double degrees, double speed){
        double cir = Diam * Math.PI;
        double tCir = WB * Math.PI;
        double dist = tCir * (degrees/360);
        double rots = dist/cir;
        double ticks = speed > 0 ? rots * TPR : -rots * TPR;
        ML.setTargetPosition(ML.getCurrentPosition() + (int) ticks);
        MR.setTargetPosition(MR.getCurrentPosition() - (int) ticks);
        ML.setPower(speed);
        MR.setPower(-speed);
    }
}
