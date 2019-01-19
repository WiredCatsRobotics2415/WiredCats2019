package frc.robot.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class PathFollowerCommand extends Command {
    private double[][] leftMotorSpeeds;
    private double[][] rightMotorSpeeds;
    private double timeLimit;
    private double time;
    private int leftTick;
    private int rightTick;
    
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

    // Called just before this Command runs the first time
    protected void initialize() {
        time = System.currentTimeMillis();
    }

    // Called repeatedly when this Command is scheduled to run
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

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if(System.currentTimeMillis()-time < timeLimit) return true;
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.velocityDrive.setMotors(0,0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        Robot.velocityDrive.setMotors(0, 0);
    }
}