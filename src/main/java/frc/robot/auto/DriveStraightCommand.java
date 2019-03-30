/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class DriveStraightCommand extends Command {
  private double distance, speed, yaw;
  private long maxTime;
  private boolean turnPID;

//                                   inches     percentage   milliseconds
  public DriveStraightCommand(double distance, double speed, long time) { 
    requires(Robot.drivetrain);
    this.distance = distance;
    this.speed = speed;
    maxTime = time;
    turnPID = true;
  }

  public DriveStraightCommand(double distance, double speed, long time, boolean turnPID) {
    this(distance, speed, time);
    this.turnPID = turnPID;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    maxTime += System.currentTimeMillis();
    yaw = Robot.drivetrain.getGryo();
    if(turnPID) {
      Robot.drivetrain.setDrivemode(Drivetrain.Drivemode.percentOutputTurnControl);
    } else {
      Robot.drivetrain.setDrivemode(Drivetrain.Drivemode.percentOutput);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(turnPID) {
      Robot.drivetrain.drive(speed, yaw);
    } else {
      Robot.drivetrain.drive(speed, 0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(System.currentTimeMillis() > maxTime) {
      return true;
    }
    if((Robot.drivetrain.getLeftEncoderPos()+Robot.drivetrain.getRightEncoderPos())/2.0 > distance) {
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.setDrivemode(Drivetrain.Drivemode.percentOutput);
    Robot.drivetrain.drive(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.drivetrain.setDrivemode(Drivetrain.Drivemode.percentOutput);
    Robot.drivetrain.drive(0, 0);
  }
}
