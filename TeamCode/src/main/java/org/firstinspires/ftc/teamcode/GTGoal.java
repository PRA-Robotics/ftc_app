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
@Autonomous(name = "NavCode V1.2.0", group = "Autonomous")
/**
 * Does stuff!
 */
public class GTGoal extends OpMode {

    //Physical constants
    /**
     * Diameter of driving wheels (cm)
     */
    final double WHEEL_DIAMETER = 9.15;
    /**
     * Distance from center of robot to a wheel (cm)
     */
    final double DIFF_DRIVE_RADIUS = 20;
    /**
     * Number of encoder ticks per driving wheel rotation
     */
    final double TICKS_PER_ROTATION = 1680.0;

    //Declare robot state:
    /**
     * Left motor object
     */
    DcMotor leftMotor;
    /**
     * Right motor object
     */
    DcMotor rightMotor;
    /**
     * Last left encoder value
     */
    int lastEncL;
    /**
     * Last right encoder value
     */
    int lastEncR;
    /**
     * Central position of the robot (X,Y)
     */
    Tuple position;
    /**
     * Direction the robot is facing (Radians)
     */
    double heading;

    //Declare autonomous parameters:
    ArrayList<Tuple> path; //Stores the robot's desired route.
    double drivingSpeed = 0.25;

    @Override
    /**
     * Initializes variable values
     */
    public void init() {
        path = new ArrayList<>(); //Creates a new empty ArrayList object
        //Add destinations to the robot's route (X,Y)
        path.add(new Tuple(85,0));
        path.add(new Tuple(85,170));
        path.add(new Tuple(-85,170));
        path.add(new Tuple(-85,0));
        path.add(new Tuple(0,0));

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
    /**
     * Follow a path
     * <p>
     *      1. Check if there is a point on the path
     *      2. If there is, move to it.
     *      3. If the target has been reached, move to the next point.
     * </p>
     */
    public void loop() {
        if (!path.isEmpty()) {
            if (moveTo(path.get(0))) {
                path.remove(0);
            }
        }
    }

    /**
     * Updates robot position and heading based off of motor rotation
     * @param deltaEncL Change in left motor
     * @param deltaEncR Change in right motor
     */
    private void updateState(int deltaEncL, int deltaEncR) {
        //Deduce the position of the left wheel based on central robot position and heading
        Tuple lPos = new Tuple(position.X + (Math.cos(heading + (Math.PI/2)) * DIFF_DRIVE_RADIUS), position.Y + (Math.sin(heading + (Math.PI/2)) * DIFF_DRIVE_RADIUS));
        //Deduce the position of the right wheel based on central robot position and heading
        Tuple rPos = new Tuple(position.X + (Math.cos(heading - (Math.PI/2)) * DIFF_DRIVE_RADIUS), position.Y + (Math.sin(heading - (Math.PI/2)) * DIFF_DRIVE_RADIUS));

        //Calculates the distance the left wheel has traveled
        double lDistance = (deltaEncL / TICKS_PER_ROTATION) * (WHEEL_DIAMETER * Math.PI);
        //Calculates the change in position (X,Y) of the left wheel based off of heading and distance
        Tuple deltaLPos = new Tuple(Math.cos(heading) * lDistance, Math.sin(heading) * lDistance);
        //Calculates the distance the right wheel has traveled
        double rDistance = (deltaEncR / TICKS_PER_ROTATION) * (WHEEL_DIAMETER * Math.PI);
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
        double dirtyHeading = Math.atan2(wheelDiff.Y,wheelDiff.X) - (Math.PI / 2.0);
        //Normalize robot heading between [0,2PI)
        heading = (dirtyHeading > 0 ? dirtyHeading : dirtyHeading + 2 * Math.PI) % (2 * Math.PI);
    }

    /**
     * Find the simplest (smallest) correction value to achieve the desired heading.
     * @param currentHeading Current heading in radians
     * @param targetHeading Desired heading in radians
     * @return Returns an angle in radians representing the minimum correction necessary to achieve the target heading
     */
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

    /**
     * Moves the robot to target position
     * @param coord target coordinates relative to starting position
     * @return True if robot has reached target, false if not
     */
    private boolean moveTo(Tuple coord) {
        //Define PID controller ratios
        final double P = 0.4;

        //Get updated encoder position for both motors
        int leftEnc = leftMotor.getCurrentPosition();
        int rightEnc = rightMotor.getCurrentPosition();

        //If the robot is farther than 0.5cm away from the target position, drive to target.
        if (position.distanceTo(coord) > 0.5) {
            //Calculate heading error and motor velocities
            double hError = headingError(heading,position.headingTo(coord));
            double lPower = drivingSpeed - (hError * P);
            double rPower = drivingSpeed + (hError * P);

            //Set motor velocities
            leftMotor.setPower(lPower);
            rightMotor.setPower(rPower);

            //Update robot state based on change in encoder position
            updateState(leftEnc-lastEncL,rightEnc-lastEncR);
            //Store current encoder values for delta calculation on next update
            lastEncL = leftEnc;
            lastEncR = rightEnc;
            return false;
        }
        // If the robot is closer than 0.5cm to its goal, stop motors and return true.
        else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            return true;
        }
    }
}

/**
 * Stores an X and a Y value (can be used as a coord or vector)
 */
class Tuple {
    /**
     * X value of the tuple object
     */
    public double X;
    /**
     * Y value of the tuple object
     */
    public double Y;
    /**
     * Creates new Tuble object with an X and a Y value
     * @param X initial X value
     * @param Y initial Y value
     */
    public Tuple(double X, double Y) {
        this.X = X;
        this.Y = Y;
    }

    /**
     * Returns the sum of a Tuple object and self
     * @param tuple Tuple object to be added to self
     * @return Sum of Tuple object tuple and self
     */
    public Tuple sum(Tuple tuple) {
        double newX = this.X + tuple.X;
        double newY = this.Y + tuple.Y;
        return new Tuple(newX,newY);
    }
    /**
     * Returns the difference of a Tuple object subtracted from self
     * @param tuple Tuple object to be subtracted from self
     * @return Difference of self and Tuple object tuple
     */
    public Tuple difference(Tuple tuple) {
        double newX = this.X - tuple.X;
        double newY = this.Y - tuple.Y;
        return new Tuple(newX,newY);
    }
    /**
     * Returns self scaled by a double
     * @param scalar scale
     * @return scaled Tuple object
     */
    public Tuple scale(double scalar) {
        double newX = this.X * scalar;
        double newY = this.Y * scalar;
        return new Tuple(newX,newY);
    }
    /**
     * Returns the difference between self (as a point) and a Tuple object (as a point)
     * @param tuple Tuple object to be used (as a point) and have its distance from self (as a point) calculated
     * @return Distance between Tuple object tuple (as a point) and self (as a point)
     */
    public double distanceTo(Tuple tuple) {
        Tuple vector = tuple.difference(this);
        return Math.sqrt(Math.pow(vector.X, 2) + Math.pow(vector.Y, 2));
    }
    /**
     * Returns the heading in radians from self (as a point) to a Tuple object (as a point)
     * @param tuple Tuple object to be used (as a point) and have its heading from self (as a point) calculated
     * @return Heading in radians from self (as a point) to Tuple object tuple (as a point)
     */
    public double headingTo(Tuple tuple) {
        Tuple vector = tuple.difference(this);
        double angle = Math.atan2(vector.Y, vector.X);
        return (angle > 0 ? angle : angle + 2 * Math.PI);
    }
}