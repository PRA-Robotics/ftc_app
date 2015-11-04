package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveMeter extends OpMode {

    Autonomous A;

    @Override
    public void init(){
        DcMotor L = hardwareMap.dcMotor.get("L");
        DcMotor R = hardwareMap.dcMotor.get("R");
        A = new Autonomous(12.7,38.1,1675,L,R);
        A.resetMotors();
    }

    @Override
    public void loop(){
        switch (A.State) {
            case 0:
                if(A.isDone(A.ML)){
                    telemetry.addData("Changing State: ",A.isDone(A.MR));
                    A.DriveDist(100, 0.5);
                    A.State++;
                }
                break;
            case 1:
                //A.TurnDegrees(180, 0.25);
                /*if(A.isDone(A.ML)){
                    A.State++;
                    telemetry.addData("Changing State","Yup");
                }*/
                break;
            case 2:
                A.DriveDist(100, 0.5);
                A.isDone(A.ML);
                break;
            case 3:
                A.TurnDegrees(180, -0.25);
                A.isDone(A.ML);
                break;
            case 4:
                A.resetMotors();
                break;
        }
        telemetry.addData("State: ", A.State);
        telemetry.addData("ML C: ", A.ML.getCurrentPosition());
        telemetry.addData("ML T: ", A.ML.getTargetPosition());
    }
}
