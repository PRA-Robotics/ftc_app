package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by FTC on 10/5/2015.
 */
public class ServoRail extends OpMode {

    double railPosition;
    boolean justPressedA;
    boolean justPressedB;
    Servo rail;

    public ServoRail(){

    }

    @Override
    public void init(){
        rail = hardwareMap.servo.get("railServo");
        railPosition = 0.5;
        justPressedA = false;
        justPressedB = false;
    }

    @Override
    public void loop(){
        if(gamepad1.right_bumper && !justPressedA){
            justPressedA = true;
            railPosition += 0.05;
        }
        if(!gamepad1.right_bumper){
            justPressedA = false;
        }
        if(gamepad1.left_bumper && !justPressedB){
            justPressedB = true;
            railPosition -= 0.05;
        }
        if(!gamepad1.left_bumper){
            justPressedB = false;
        }
        railPosition = Range.clip(railPosition, 0, 1);
        rail.setPosition(railPosition);
        telemetry.addData("jpA", justPressedA);
        telemetry.addData("jpB", justPressedB);
        telemetry.addData("RailPos", railPosition);
    }
}
