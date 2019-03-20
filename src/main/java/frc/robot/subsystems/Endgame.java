/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Endgame extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSolenoid endgameSolenoid;

  public Endgame() {
    endgameSolenoid = new DoubleSolenoid(RobotMap.PCM_ID, RobotMap.CLIMB_PISTON_1, RobotMap.CLIMB_PISTON_2);
  }

  public void flipOut() {
    // lSwitch.set(Value.kForward);
    endgameSolenoid.set(Value.kForward);
  }

  public void flipIn() {
    // lSwitch.set(Value.kReverse);
    endgameSolenoid.set(Value.kReverse);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
