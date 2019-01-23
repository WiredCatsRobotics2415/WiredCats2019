package frc.robot.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Command class for the robot to follow a predetermined path
 */
public class PathFollowerCommand extends Command {
    /**
     * 2d array to hold left motor speeds [time (milliseconds)][velocity]
     */
    private double[][] leftMotorSpeeds;
    /**
     * 2d array to hold right motor speeds [time (milliseconds)][velocity]
     */
    private double[][] rightMotorSpeeds;
    /**
     * max time for the command to runn
     */
    private double timeLimit;
    /**
     * time when command started
     */
    private double time;
    /**
     * current array location for left motor speeds
     */
    private int leftTick;
    /**
     * current array location for right motor speeds
     */
    private int rightTick;
    
    /**
     * set the motor speeds for the motors and instantiates other varibles
     * @param leftMotorSpeeds speeds for left motors [time(milliseconnds)][velocity]
     * @param rightMotorSpeeds speeds for right motors [time(milliseconnds)][velocity]
     */
    public PathFollowerCommand(double[][] leftMotorSpeeds, double[][] rightMotorSpeeds) {
        this.leftMotorSpeeds = leftMotorSpeeds;
        this.rightMotorSpeeds = rightMotorSpeeds;
        if(leftMotorSpeeds[leftMotorSpeeds.length-1][0] < rightMotorSpeeds[rightMotorSpeeds.length-1][0]) {
            this.timeLimit = leftMotorSpeeds[leftMotorSpeeds.length-1][0];
        } else {
            this.timeLimit = rightMotorSpeeds[rightMotorSpeeds.length-1][0];
        }
        this.leftTick = 0;
        this.rightTick = 0;
        requires(Robot.velocityDrive);
    }

    /**
     * Method called just before this Command runs the first time
     */
    protected void initialize() {
        time = System.currentTimeMillis();
    }

    /**
     * Method called repeatedly when this Command is scheduled to run
     * <p>
     * sets the motors to the predetermined speed for the time
     * </p>
     */
    protected void execute() {
        double timeAway = Double.MAX_VALUE;
        while(leftTick < leftMotorSpeeds.length && Math.abs(leftMotorSpeeds[leftTick][0]-(System.currentTimeMillis()+time)) < timeAway) {
            leftTick++;
        }
        while(rightTick < rightMotorSpeeds.length && Math.abs(rightMotorSpeeds[rightTick][0]-(System.currentTimeMillis()+time)) < timeAway) {
            rightTick++;
        }
        Robot.velocityDrive.setMotors(leftMotorSpeeds[leftTick][1], rightMotorSpeeds[rightTick][1]);
    }

    /**
     * returns true when this Command no longer needs to run execute()
     * @return if the command is finished
     */
    protected boolean isFinished() {
        if(System.currentTimeMillis()-time < timeLimit) return true;
        return false;
    }

    /**
     * Method called once after isFinished returns true
     */
    protected void end() {
        Robot.velocityDrive.setMotors(0,0);
    }

    /**
     * Method called when another command which requires one or more of the same subsystems is scheduled to run
     */
    protected void interrupted() {
        Robot.velocityDrive.setMotors(0, 0);
    }
}