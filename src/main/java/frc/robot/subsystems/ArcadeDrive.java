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
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.cheesy.DriveSignal;

/**
 * Add your docs here.
 */
public class ArcadeDrive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX lFront, rFront, lBack, rBack;
  public AHRS ahrs;

  private final double WHEEL_CIRCUMFERENCE = Math.PI * 8; //inches
  public final double DEADBAND = 0.05;

	public final float INTERPOLATION_FACTOR = 0.75f;   //Nathan's Settings
	public final float STRAIGHT_LIMITER = 0.95f;
	public final float TURN_BOOSTER = 1.3f;


  public ArcadeDrive() {
    lFront = new WPI_TalonSRX(RobotMap.LEFT_TALON_FRONT);
    rFront = new WPI_TalonSRX(RobotMap.RIGHT_TALON_FRONT);
    lBack = new WPI_TalonSRX(RobotMap.LEFT_TALON_BACK);
    rBack = new WPI_TalonSRX(RobotMap.RIGHT_TALON_BACK);

    // try {
    //   ahrs = new AHRS(Port.kMXP);
    // } catch (RuntimeException ex) {
    //   DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    // }

    lFront.setInverted(true);
    lBack.setInverted(true);
    rFront.setInverted(false);
    rBack.setInverted(false);


    lFront.set(ControlMode.Follower, lBack.getDeviceID());
    rFront.set(ControlMode.Follower, rBack.getDeviceID());

    lBack.set(ControlMode.PercentOutput, 0);
    rBack.set(ControlMode.PercentOutput, 0);

    lFront.set(ControlMode.PercentOutput, 0);
    rFront.set(ControlMode.PercentOutput, 0);

    lFront.setNeutralMode(NeutralMode.Coast);
    rFront.setNeutralMode(NeutralMode.Coast);

  }

  public void testMotor(double vel) {
    lFront.set(vel);
    // System.out.println("percentage"+lFront.getMotorOutputPercent());
    // System.out.println("voltage"+lFront.getMotorOutputVoltage());
    } 

  public void setMotors(double left, double right) {
    lBack.set(left);
    rBack.set(right);
  }

  public void drive(DriveSignal signal) {
    setMotors(signal.getLeft(), signal.getRight());
  }

  public void zeroYaw() {
    ahrs.zeroYaw();
  }

  public double getYaw() {
    return ahrs.getYaw();
  }
  
  public void setBrakeMode(boolean brake) {
    /*if (brake) {
      lBack.setNeutralMode(NeutralMode.Brake);
      rBack.setNeutralMode(NeutralMode.Brake);
    } else {
      lBack.setNeutralMode(NeutralMode.Coast);
      rBack.setNeutralMode(NeutralMode.Coast);
    }*/
  }

  public void printCurrent() {
    //lFront, rFront, lBack, rBack, lMid, rMid;
    System.out.println("lFront current="+lFront.getOutputCurrent());
    System.out.println("lBack current="+lBack.getOutputCurrent());
    System.out.println("rFront current="+rFront.getOutputCurrent());
    System.out.println("rBack current="+rBack.getOutputCurrent());
    System.out.println();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
