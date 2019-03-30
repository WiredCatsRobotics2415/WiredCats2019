/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class FrontCargoShip extends CommandGroup {
  public static final double sideOffset = 48; //inches
  public static final double distanceToShip = 60;
  public static final double targetXOffset = 12;
  public static final double straightOnDistance = 24;

  public FrontCargoShip(Robot.StartLocation startPos, boolean left) {
    if(left) {
      switch(startPos) {
        case left: //left
          addSequential(new TurnToAngleCommand(Math.toDegrees(Math.atan2(sideOffset-targetXOffset, distanceToShip-straightOnDistance)), 5000, .4, false, false));
          break;
        case middle: //middle
          addSequential(new TurnToAngleCommand(Math.toDegrees(-Math.atan2(targetXOffset, distanceToShip-straightOnDistance)), 5000, .4, false, false));
          break; 
        case right: //right
          addSequential(new TurnToAngleCommand(Math.toDegrees(-Math.atan2(sideOffset+targetXOffset, distanceToShip-straightOnDistance)), 5000, .4, false, false));
          break;
      }
    } else {
      switch(startPos) {
        case left: //left
          addSequential(new TurnToAngleCommand(Math.toDegrees(Math.atan2(sideOffset+targetXOffset, distanceToShip-straightOnDistance)), 5000, .4, false, false));
          break;
        case middle: //middle
          addSequential(new TurnToAngleCommand(Math.toDegrees(Math.atan2(targetXOffset, distanceToShip-straightOnDistance)), 5000, .4, false, false));
          break; 
        case right: //right
          addSequential(new TurnToAngleCommand(Math.toDegrees(-Math.atan2(sideOffset-targetXOffset, distanceToShip-straightOnDistance)), 5000, .4, false, false));
          break;
      }
    }
    if(left) {
      switch(startPos) {
        case left: //left
          addSequential(new DriveStraightCommand(pythag(sideOffset-targetXOffset, distanceToShip-straightOnDistance), .6, 5000, false));
          break;
        case middle: //middle
          addSequential(new DriveStraightCommand(pythag(targetXOffset, distanceToShip-straightOnDistance), .6, 5000, false));
          break; 
        case right: //right
          addSequential(new DriveStraightCommand(pythag(sideOffset+targetXOffset, distanceToShip-straightOnDistance), .6, 5000, false));
          break;
      }
    } else {
      switch(startPos) {
        case left: //left
          addSequential(new DriveStraightCommand(pythag(sideOffset+targetXOffset, distanceToShip-straightOnDistance), .6, 5000, false));
          break;
        case middle: //middle
          addSequential(new DriveStraightCommand(pythag(targetXOffset, distanceToShip-straightOnDistance), .6, 5000, false));
          break; 
        case right: //right
          addSequential(new DriveStraightCommand(pythag(sideOffset-targetXOffset, distanceToShip-straightOnDistance), .6, 5000, false));
          break;
      }
    }
    if(left) {
      switch(startPos) {
        case left: //left
          addSequential(new TurnToAngleCommand(Math.toDegrees(-Math.atan2(sideOffset-targetXOffset, distanceToShip-straightOnDistance)), 5000, .4, false, false));
          break;
        case middle: //middle
          addSequential(new TurnToAngleCommand(Math.toDegrees(Math.atan2(targetXOffset, distanceToShip-straightOnDistance)), 5000, .4, false, false));
          break; 
        case right: //right
          addSequential(new TurnToAngleCommand(Math.toDegrees(Math.atan2(sideOffset+targetXOffset, distanceToShip-straightOnDistance)), 5000, .4, false, false));
          break;
      }
    } else {
      switch(startPos) {
        case left: //left
          addSequential(new TurnToAngleCommand(Math.toDegrees(-Math.atan2(sideOffset+targetXOffset, distanceToShip-straightOnDistance)), 5000, .4, false, false));
          break;
        case middle: //middle
          addSequential(new TurnToAngleCommand(Math.toDegrees(-Math.atan2(targetXOffset, distanceToShip-straightOnDistance)), 5000, .4, false, false));
          break; 
        case right: //right
          addSequential(new TurnToAngleCommand(Math.toDegrees(Math.atan2(sideOffset-targetXOffset, distanceToShip-straightOnDistance)), 5000, .4, false, false));
          break;
      }
    }
    addSequential(new DriveStraightCommand(straightOnDistance, .5, 5000, false));
  }

  public double pythag(double a, double b) {
    return Math.sqrt(Math.pow(a,2)+Math.pow(b,2));
  }
}
