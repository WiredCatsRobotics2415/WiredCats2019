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

public class TurnToAngleCommand extends Command {
  public static final double TOLERANCE = 2.0;
  private double targetAngle;
  private double speed;
  private long maxTime;
  private boolean pidTurn;

  public TurnToAngleCommand(double targetAngle, long time) {
    requires(Robot.drivetrain);
    this.targetAngle = targetAngle;
    this.maxTime = time;
    this.pidTurn = true;
    this.speed = 0;
  }

  public TurnToAngleCommand(double targetAngle, long time, double speed, boolean pidTurn) {
    this(targetAngle, time);
    this.pidTurn = pidTurn;
    this.speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    maxTime += System.currentTimeMillis();
    targetAngle += Robot.drivetrain.getGryo();
    if(pidTurn) {
      Robot.drivetrain.setDrivemode(Drivetrain.Drivemode.percentOutputTurnControl);
    } else {
      Robot.drivetrain.setDrivemode(Drivetrain.Drivemode.percentOutput);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(pidTurn) {
      Robot.drivetrain.drive(0, targetAngle);
    } else {
      if(Robot.drivetrain.getGryo() < targetAngle) {
        Robot.drivetrain.drive(0,speed);
      } else {
        Robot.drivetrain.drive(0,-speed);
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Math.abs(Robot.drivetrain.getGryo()-targetAngle) < TOLERANCE) {
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
