package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import java.util.HashMap;

public class Autonomous extends LinearOpMode {
    private HashMap<String,HardwareDevice> HWDS;
    private double Diam;
    private double WB;
    private double TPR;
    private DcMotor ML;
    private DcMotor MR;
    private int LT;
    private int RT;

    public Autonomous (double wheelDiameter, double wheelBase, double ticksPerRotation, DcMotor leftMotor, DcMotor rightMotor){
        HWDS = new HashMap<String,HardwareDevice>();
        Diam = wheelDiameter;
        WB = wheelBase;
        TPR = ticksPerRotation;
        ML = leftMotor;
        MR = rightMotor;
        ML.setDirection(DcMotor.Direction.REVERSE);
        ML.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        MR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    private boolean isDone(DcMotor M, int T){
        int current = M.getCurrentPosition();
        int diff = Math.abs(T-current);
        if(diff <= 10) {
            return (true);
        }
        else {
            return (false);
        }
    }

    private void waitForPos() throws InterruptedException {
        boolean DL;
        do {
            waitForNextHardwareCycle();
            DL = isDone(ML,LT);
        } while(!DL); // Add code for right motor as well.
        ML.setPower(0);
        MR.setPower(0);
    }

    public void addDevice(String deviceName, UltrasonicSensor device) {
        HWDS.put(deviceName, device);
    }

    public UltrasonicSensor getUSS(String deviceName) {
        return ((UltrasonicSensor) HWDS.get(deviceName));
    }

    public void driveToDist (double targetDist, double speed, String sensorID) throws InterruptedException {
        ML.setPower(speed);
        MR.setPower(speed);
        UltrasonicSensor USS = getUSS(sensorID);
        double diff = Math.abs(targetDist - USS.getUltrasonicLevel());
        while(diff <= 3){waitForNextHardwareCycle();};
        stopRobot();
    }

    public void DriveDist (double distance, double speed) throws InterruptedException {
        waitForPos();
        double cir = Diam * Math.PI;
        double rots = distance/cir;
        double ticks = speed > 0 ? rots * TPR : -rots * TPR;
        LT += (int) ticks;
        RT += (int) ticks;
        ML.setPower(speed);
        MR.setPower(speed);
    }

    public void TurnDegrees (double degrees, double speed) throws InterruptedException {
        waitForPos();
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

    public void stopRobot() throws InterruptedException {
        waitForPos();
    }

    @Override
    public void runOpMode() throws InterruptedException {}
}
