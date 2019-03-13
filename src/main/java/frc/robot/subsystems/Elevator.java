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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import java.util.ArrayList;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX elevOne, elevTwo, elevThree, elevFour;
  private DoubleSolenoid shifter;
  private DigitalInput bottom, top;
  
  private WPI_TalonSRX elevMaster;

  public Elevator() {
    elevOne = new WPI_TalonSRX(RobotMap.ELEVATOR_ONE);
    elevTwo = new WPI_TalonSRX(RobotMap.ELEVATOR_TWO);
    elevThree = new WPI_TalonSRX(RobotMap.ELEVATOR_THREE);
    elevFour = new WPI_TalonSRX(RobotMap.ELEVATOR_FOUR);

    shifter = new DoubleSolenoid(RobotMap.PCM_ID, RobotMap.ELEV_SWITCH_1, RobotMap.ELEV_SWITCH_2);

    elevOne.setInverted(RobotMap.ELEVATOR_ONE_DIRECTION);
    elevTwo.setInverted(RobotMap.ELEVATOR_TWO_DIRECTION);
    elevThree.setInverted(RobotMap.ELEVATOR_THREE_DIRECTION);
    elevFour.setInverted(RobotMap.ELEVATOR_FOUR_DIRECTION);

    top = new DigitalInput(RobotMap.ELEV_TOP);
    bottom = new DigitalInput(RobotMap.ELEV_BOT);

    elevTwo.follow(elevOne);
    elevThree.follow(elevOne);
    elevFour.follow(elevOne);

    elevMaster = elevOne;
  
    elevOne.setNeutralMode(NeutralMode.Brake);
    elevOne.set(ControlMode.PercentOutput, 0);

  }

  public void setBrakeMode(boolean brake) {
    if (brake) {
      elevMaster.setNeutralMode(NeutralMode.Brake);
    } else {
      elevMaster.setNeutralMode(NeutralMode.Coast);
    }
  }

  public boolean getTop() {
    return top.get();
  }

  public boolean getBottom() {
    return bottom.get();
  }

  public void setElevMotors(double speed) {
    elevMaster.set(speed);
  }

  public void liftUp() {
    elevMaster.set(0.5);
  }

  public void lowerDown() {
    elevMaster.set(-0.2);
  }

  public void stop() {
    elevMaster.set(0);
  }

  public void shift() {
    if (shifter.get() == Value.kForward) {
      shifter.set(Value.kReverse);
    } else {
      shifter.set(Value.kForward);
    }
  }

  public void shiftUp() {
    shifter.set(Value.kReverse);
  }

  public void shiftDown() {
    shifter.set(Value.kForward);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}