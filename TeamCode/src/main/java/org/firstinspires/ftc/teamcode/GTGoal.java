package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.ArrayList;

/*
TODO:
  * Clean up this mess.
  * Move to a proportional or PID controller for movement, and make driving and heading adjustments one in the same.
  * Split Coordinate and Vector functionality into separate classes.
*/
@Autonomous(name = "NavCode V1.0.0", group = "Autonomous")
public class GTGoal extends OpMode {

    //Constants
    double wheelDiameter = 9.15; //Diameter of driving wheels (cm)
    double diffDriveRadius = 20; //Distance from center of robot to a wheel (cm)
    double ticksPerRotation = 1680.0; //Number of encoder ticks per driving wheel rotation

    //Declare robot variables:
    DcMotor leftMotor; //Left motor object
    DcMotor rightMotor; //Right motor object
    int lastEncL; //Last left encoder value
    int lastEncR; //Last right encoder value
    Tuple position; //Central position of the robot (X,Y)
    double heading; //Direction the robot is facing (Radians)

    ArrayList<Tuple> path; //Stores the robot's desired route.

    @Override
    public void init() {
        path = new ArrayList<>(); //Creates a new empty ArrayList object
        path.add(new Tuple(100,0)); //Adds a destination to the robot's route (X,Y)

        leftMotor = hardwareMap.dcMotor.get("L"); //Set 'leftMotor' to the motor 'L' from the HardwareMap
        rightMotor = hardwareMap.dcMotor.get("R"); //Set 'rightMotor' to the motor 'R' from the HardwareMap
        rightMotor.setDirection(DcMotor.Direction.REVERSE); //Reverses the right motor so both motors drive forward
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Stops robot and resets encoder
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Stops robot and resets encoder
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Put motor back into driving mode
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Put motor back into driving mode
        lastEncL = leftMotor.getCurrentPosition(); //Sets original encoder value to current
        lastEncR = rightMotor.getCurrentPosition(); //Sets original encoder value to current
        position = new Tuple(0,0); //Sets starting position to (0,0)
        heading = 0; //Set starting heading to 0 radians
    }

    @Override
    //1. Check if there is a point on the path
    //2. If there is, move to it.
    //3. If the target has been reached, move to the next point.
    public void loop() {
        if (!path.isEmpty()) {
            if (moveTo(path.get(0))) {
                path.remove(0);
            }
        }
    }

    //Updates robot position and heading based off of motor rotation
    private void updateState(int deltaEncL, int deltaEncR) {
        //Deduce the position of the left wheel based on central robot position and heading
        Tuple lPos = new Tuple(position.X + (Math.cos(heading + (Math.PI/2)) * diffDriveRadius), position.Y + (Math.sin(heading + (Math.PI/2)) * diffDriveRadius));
        //Deduce the position of the right wheel based on central robot position and heading
        Tuple rPos = new Tuple(position.X + (Math.cos(heading - (Math.PI/2)) * diffDriveRadius), position.Y + (Math.sin(heading - (Math.PI/2)) * diffDriveRadius));

        //Calculates the distance the left wheel has traveled
        double lDistance = (deltaEncL / ticksPerRotation) * (wheelDiameter * Math.PI);
        //Calculates the change in position (X,Y) of the left wheel based off of heading and distance
        Tuple deltaLPos = new Tuple(Math.cos(heading) * lDistance, Math.sin(heading) * lDistance);
        //Calculates the distance the right wheel has traveled
        double rDistance = (deltaEncR / ticksPerRotation) * (wheelDiameter * Math.PI);
        //Calculates the change in position (X,Y) of the right wheel based off of heading and distance
        Tuple deltaRPos = new Tuple(Math.cos(heading) * rDistance, Math.sin(heading) * rDistance);

        //Updates the left wheel position based off of the change in position
        lPos = lPos.sum(deltaLPos);
        //Updates the right wheel position based off of the change in position
        rPos = rPos.sum(deltaRPos);
        //Finds the average X position of the robot
        position.X = (lPos.X + rPos.X) / 2;
        //Finds the average Y position of the robot
        position.Y = (lPos.Y + rPos.Y) / 2;

        //Calculate dirty (non-normalized) heading based on difference in wheel positions
        Tuple wheelDiff = lPos.difference(rPos);
        double dirtyHeading = Math.atan2(wheelDiff.X,wheelDiff.Y);
        //Normalize robot heading between [0,2PI)
        heading = (heading + (dirtyHeading > 0 ? dirtyHeading : dirtyHeading + 2 * Math.PI)) % (2 * Math.PI);
    }

    //Find the simplest (smallest) correction value to achieve the desired heading.
    private double headingError(double currentHeading, double targetHeading) {
        //Find the difference between the target and current headings
        double error = targetHeading - currentHeading;
        //If the difference is greater than 180 degrees, use the negative angle instead
        if (error > Math.PI) {
            return error - (2 * Math.PI);
        }
        //If the difference is smaller than -180 degrees, use the positive angle instead
        if (error < -Math.PI) {
            return error + (2 * Math.PI);
        }
        return error;
    }

    //Navigate to target point and return true if target has been reached
    private boolean moveTo(Tuple coord) {
        //Get updated encoder position for both motors
        int leftEnc = leftMotor.getCurrentPosition();
        int rightEnc = rightMotor.getCurrentPosition();

        //If the robot is farther than 0.5cm away from the target position, drive to target.
        if (position.distanceTo(coord) > 0.5) {
            //Set motor power to 50% for both motors
            leftMotor.setPower(0.5);
            rightMotor.setPower(0.5);
            //Update robot state based on change in encoder position
            updateState(leftEnc-lastEncL,rightEnc-lastEncR);
            //Store current encoder values for delta calculation on next update
            lastEncL = leftEnc;
            lastEncR = rightEnc;
            return false;
            //--------------- Insert PID Controller Here ---------------
            /*
            double hError = headingError(heading,position.headingTo(coord));
            if (Math.abs(hError) > 0.05) {
                if (Math.abs(hError) < 0.1) {
                    if (hError > 0) {
                        leftMotor.setPower(0.3);
                        rightMotor.setPower(0.4);
                    }
                    if (hError < 0) {
                        leftMotor.setPower(0.4);
                        rightMotor.setPower(0.3);
                    }
                }
                else {
                    if (hError > 0) {
                        leftMotor.setPower(-0.4);
                        rightMotor.setPower(0.4);
                    }
                    if (hError < 0) {
                        leftMotor.setPower(0.4);
                        rightMotor.setPower(-0.4);
                    }
                }
                updateState(leftEnc-lastEncL,rightEnc-lastEncR);
                lastEncL = leftEnc;
                lastEncR = rightEnc;
                return false;
            }
            else {
                leftMotor.setPower(0.4);
                rightMotor.setPower(0.4);
                updateState(leftEnc-lastEncL,rightEnc-lastEncR);
                lastEncL = leftEnc;
                lastEncR = rightEnc;
                return false;
            }*/
            //--------------- End PID Block ---------------
        }
        // If the robot is closer than 0.5cm to its goal, stop motors and return true.
        else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            return true;
        }
    }
}

class Tuple {
    public double X;
    public double Y;

    public Tuple(double X, double Y) {
        this.X = X;
        this.Y = Y;
    }

    public Tuple sum(Tuple tuple) {
        double newX = this.X + tuple.X;
        double newY = this.Y + tuple.Y;
        return new Tuple(newX,newY);
    }

    public Tuple difference(Tuple tuple) {
        double newX = this.X - tuple.X;
        double newY = this.Y - tuple.Y;
        return new Tuple(newX,newY);
    }

    public Tuple scale(double scalar) {
        double newX = this.X * scalar;
        double newY = this.Y * scalar;
        return new Tuple(newX,newY);
    }

    public double distanceTo(Tuple tuple) {
        Tuple vector = tuple.difference(this);
        return Math.sqrt(Math.pow(vector.X, 2) + Math.pow(vector.Y, 2));
    }

    public double headingTo(Tuple tuple) {
        Tuple vector = tuple.difference(this);
        double angle = Math.atan2(vector.Y, vector.X);
        return (angle > 0 ? angle : angle + 2 * Math.PI);
    }
}