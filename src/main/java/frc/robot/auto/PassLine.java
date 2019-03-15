/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

import frc.robot.Robot;

public class PassLine extends CommandGroup {

  public PassLine(Robot.StartLocation startPos) {
    switch(startPos) {
      case left:
        addSequential(new TurnToAngleCommand(-20, 5000, .3, false));
        addSequential(new DriveStraightCommand(96,.7,5000,false));
        break;
      case right:
        addSequential(new TurnToAngleCommand(20, 5000, .3, false));
        addSequential(new DriveStraightCommand(96,.7,5000,false));
        break;
      case middle:
        addSequential(new WaitCommand(5));
        addSequential(new DriveStraightCommand(60,.7,5000,false));
        break;
      case left_Level_2:
        addSequential(new DriveStraightCommand(48, .7, 5000, false));
        addSequential(new TurnToAngleCommand(-25, 5000, .3, false));
        addSequential(new DriveStraightCommand(96,.7,5000,false));
        break;
      case right_Level_2:
        addSequential(new DriveStraightCommand(48, .7, 5000, false));
        addSequential(new TurnToAngleCommand(25, 5000, .3, false));
        addSequential(new DriveStraightCommand(96,.7,5000,false));
        break;
      default:
        addSequential(new DriveStraightCommand(60,.7,5000,false));
        break;
    }
  }
}
