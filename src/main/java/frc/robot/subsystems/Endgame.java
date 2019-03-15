/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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

  private DoubleSolenoid lSwitch, rSwitch;
  private WPI_TalonSRX lClimb, rClimb, masterClimb;

  public Endgame() {
    // lSwitch = new DoubleSolenoid(RobotMap.PCM_ID, RobotMap.CLIMB_LEFT_PISTON_1, RobotMap.CLIMB_LEFT_PISTON_2);
    rSwitch = new DoubleSolenoid(RobotMap.PCM_ID, RobotMap.CLIMB_RIGHT_PISTON_1, RobotMap.CLIMB_RIGHT_PISTON_2);

    lClimb = new WPI_TalonSRX(RobotMap.CLIMB_LEFT_MOTOR);
    // rClimb = new WPI_TalonSRX(RobotMap.CLIMB_RIGHT_MOTOR);

    lClimb.setInverted(RobotMap.CLIMB_LEFT_DIRECTION);
    // rClimb.setInverted(RobotMap.CLIMB_RIGHT_DIRECTION);

    // rClimb.follow(lClimb);

    masterClimb = lClimb;
    masterClimb.setNeutralMode(NeutralMode.Brake);
    masterClimb.set(ControlMode.PercentOutput, 0);

  }

  public void testMotor(double speed) {
    lClimb.set(speed);
  }

  public void stop() {
    masterClimb.set(0);
  }

  public void spin() {
    masterClimb.set(0.8);
  }

  public void flipOut() {
    // lSwitch.set(Value.kForward);
    rSwitch.set(Value.kForward);
  }

  public void flipIn() {
    // lSwitch.set(Value.kReverse);
    rSwitch.set(Value.kReverse);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
