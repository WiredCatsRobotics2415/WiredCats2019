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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX elevOne, elevTwo, elevThree, elevFour;
  private Solenoid shifter;
  private WPI_TalonSRX[] elevTalons;
  private DigitalInput lowBall, lowHatch, midBall, midHatch, highBall, highHatch;

  public Elevator() {
    elevOne = new WPI_TalonSRX(RobotMap.ELEVATOR_ONE);
    elevTwo = new WPI_TalonSRX(RobotMap.ELEVATOR_TWO);
    elevThree = new WPI_TalonSRX(RobotMap.ELEVATOR_THREE);
    elevFour = new WPI_TalonSRX(RobotMap.ELEVATOR_FOUR);

    shifter = new Solenoid(RobotMap.PCM_ID);

    WPI_TalonSRX[] elevTalons = {elevOne, elevTwo, elevThree, elevFour};

    elevTwo.set(ControlMode.Follower, elevOne.getDeviceID());
    elevThree.set(ControlMode.Follower, elevOne.getDeviceID());
    elevFour.set(ControlMode.Follower, elevOne.getDeviceID());
    
    elevOne.setNeutralMode(NeutralMode.Brake);
    elevOne.set(ControlMode.PercentOutput, 0);
  }

  public void liftUp() {
    for (WPI_TalonSRX talon: elevTalons) {
      talon.set(0.5);
    }
  }

  public void lowerDown() {
    for (WPI_TalonSRX talon: elevTalons) {
      talon.set(-0.3);
    }
  }

  public void shift() {
    if (shifter.get()) {
      shifter.set(false);
    } else {
      shifter.set(true);
    }
  }

  public void aim(DigitalInput goal) {
    if (goal == this.lowBall) {

    } else if (goal == this.lowHatch) {

    } else if (goal == this.midBall) {

    } else if (goal == this.midHatch) {

    } else if (goal == this.highBall) {

    } else if (goal == this.highHatch) {

    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
