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

import java.util.ArrayList;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX elevOne, elevTwo, elevThree, elevFour;
  private Solenoid shifter;
  private WPI_TalonSRX[] elevTalons;
  private DigitalInput bottom, lowBall, lowHatch, midBall, midHatch, highBall, highHatch;
  public byte lastHeight, B, LB, LH, MB, MH, HB, HH, T;
  public ArrayList<Byte> targets;

  public Elevator(byte lastHeight) {
    elevOne = new WPI_TalonSRX(RobotMap.ELEVATOR_ONE);
    elevTwo = new WPI_TalonSRX(RobotMap.ELEVATOR_TWO);
    elevThree = new WPI_TalonSRX(RobotMap.ELEVATOR_THREE);
    elevFour = new WPI_TalonSRX(RobotMap.ELEVATOR_FOUR);

    shifter = new Solenoid(RobotMap.PCM_ID);

    bottom = new DigitalInput(0);

    WPI_TalonSRX[] elevTalons = {elevOne, elevTwo, elevThree, elevFour};
    ArrayList<Byte> targets = new ArrayList<Byte>();
    targets.add(B);
    targets.add(LB);
    targets.add(LH);

    elevTwo.set(ControlMode.Follower, elevOne.getDeviceID());
    elevThree.set(ControlMode.Follower, elevOne.getDeviceID());
    elevFour.set(ControlMode.Follower, elevOne.getDeviceID());
    
    elevOne.setNeutralMode(NeutralMode.Brake);
    elevOne.set(ControlMode.PercentOutput, 0);

    lastHeight = B;
    this.lastHeight = lastHeight;
  }

  public void setElevMotors(double speed) {
    for (WPI_TalonSRX talon: elevTalons) {
      talon.set(speed);
    }
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

  public void stop() {
    for (WPI_TalonSRX talon: elevTalons) {
      talon.set(0);
    }
  }

  public void shift() {
    if (shifter.get()) {
      shifter.set(false);
    } else {
      shifter.set(true);
    }
  }

  public boolean above(byte last, byte target) {
    if (targets.indexOf(last) > targets.indexOf(target)) {
      return false;
    } else {
      return true;
    }
  }

  public void aim(byte goal) {
    if (goal == LB) {
      if (!lowBall.get()) {
        liftUp();
      } else {
        stop();
      }
    } else if (goal == LH) {

    } else if (goal == MB) {

    } else if (goal == MH) {

    } else if (goal == HB) {

    } else if (goal == HH) {

    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
