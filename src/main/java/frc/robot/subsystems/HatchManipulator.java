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

  private Solenoid extendyBoi, stretchyBoi;

  public HatchManipulator() {
    extendyBoi = new Solenoid(RobotMap.PCM_ID, RobotMap.EXTENDY);
    stretchyBoi = new Solenoid(RobotMap.PCM_ID, RobotMap.STRETCHY);
  }

  public void extend() {
    extendyBoi.set(true);
  }

  public void retract() {
    extendyBoi.set(false);
  }

  public void extendToggle() {
    extendyBoi.set(!extendyBoi.get());
  }

  public void stretch() {
    stretchyBoi.set(true);
  }

  public void shrink() {
    stretchyBoi.set(false);
  }

  public void stretchToggle() {
    stretchyBoi.set(!stretchyBoi.get());
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
