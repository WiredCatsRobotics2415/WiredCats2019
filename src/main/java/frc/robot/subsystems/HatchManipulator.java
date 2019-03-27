/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class HatchManipulator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSolenoid extendyBoi, stretchyBoi;

  public HatchManipulator() {
    extendyBoi = new DoubleSolenoid(RobotMap.PCM_ID, RobotMap.EXTENDY_1, RobotMap.EXTENDY_2);
    stretchyBoi = new DoubleSolenoid(RobotMap.PCM_ID, RobotMap.STRETCHY_1, RobotMap.STRETCHY_2);
  }

  public boolean isOut() {
    if (stretchyBoi.get() == Value.kForward) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isStretched() {
    if (extendyBoi.get() == Value.kForward) {
      return true;
    } else {
      return false;
    }
  }

  public void extend() {
    extendyBoi.set(Value.kForward);
  }

  public void retract() {
    extendyBoi.set(Value.kReverse);
  }

  public void extendToggle() {
    if (extendyBoi.get() == Value.kForward) {
      extendyBoi.set(Value.kReverse);
    } else {
    extendyBoi.set(Value.kForward);
    }
    
  }

  public void stretch() {
    stretchyBoi.set(Value.kForward);
  }

  public void shrink() {
    stretchyBoi.set(Value.kReverse);
  }

  public void stretchToggle() {
    if (stretchyBoi.get() == Value.kForward) {
      stretchyBoi.set(Value.kReverse);
    } else {
      stretchyBoi.set(Value.kForward);
    }
  }

  public void takeHatch() {

  }

  public void placeHatch() {

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
