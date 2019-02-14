package frc.robot.util;

/**
 * Class to hold to values for the drivetrain
 * 
 * @author Matthew Propp
 */
public class DriveSignal {
    /**
     * speed of the right side ranging from 0.0 to 1.0
     */
    private double rightSignal;
    /**
     * speed of the left side ranging from 0.0 to 1.0
     */
    private double leftSignal;
    private double speed;
    private double r;

    /**
     * Takes the speed and rotation input and determines the speed to set each motor
     * @param speed forward speed ranging from -1.0 to 1.0
     * @param r clockwise rotation ranging from -1.0 to 1.0
     */
    public DriveSignal(double speed, double r) {
        rightSignal = speed + r;
        leftSignal = speed - r;
        this.speed = speed;
        this.r = r;
    }

    /**
     * Takes the speed and rotation input and determines the speed to set each motor
     * @param speed forward speed ranging from -1.0 to 1.0
     * @param r clockwise rotation ranging from -1.0 to 1.0
     * @param quickTurn if the robot should only turn
     */
    public DriveSignal(double speed, double r, boolean quickTurn) {
        this(speed,r);
        if(quickTurn) {
            rightSignal = r;
            leftSignal = -r;
            this.speed = 0;
        }
    }

    /**
     * Get the speed for the right motor
     * @return the speed of the right motors ranging from -1.0 to 1.0
     */
    public double getRight() {
        return rightSignal;
    }

    /**
     * Get the speed for the left motor
     * @return the speed of the left motors ranging from -1.0 to 1.0
     */
    public double getLeft() {
        return leftSignal;
    }

    public double getSpeed() {
        return speed;
    }
    
    public double getR() {
        return r;
    }
}
