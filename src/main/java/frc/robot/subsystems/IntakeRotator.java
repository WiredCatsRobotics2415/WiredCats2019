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

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class IntakeRotator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX rotator;

  public IntakeRotator() {
    rotator = new WPI_TalonSRX(RobotMap.INTAKE_ROTATE);

    rotator.setInverted(true);

    rotator.set(ControlMode.PercentOutput, 0);
    rotator.setNeutralMode(NeutralMode.Brake);

  }

  public void rotateUp() {
    rotator.set(-1);
  }

  public void rotateDown() {
    rotator.set(0.5);
  }

  public void still() {
    rotator.set(0);
  }

  public void setMotor(double speed) {
    rotator.set(speed);
  }

  public void setBrakeMode(boolean brake) {
    if (brake) {
      rotator.setNeutralMode(NeutralMode.Brake);
    } else {
      rotator.setNeutralMode(NeutralMode.Coast);
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
