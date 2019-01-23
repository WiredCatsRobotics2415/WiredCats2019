/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.util.DriveSignal;

/**
 * Subsystem to control the drivetrain based on velocity
 * @author Matthew Propp
 */
public class VelocityDrive extends Subsystem {
  /**
   * The motors for the drivetrain
   */
  private WPI_TalonSRX lFront, rFront, lBack, rBack;
  /**
   * Accelerometer
   */
  public AHRS ahrs;
  /**
   * A constant to hold the cirumference of the wheel in inches for distance calculations
   */
  private final double WHEEL_CIRCUMFERENCE = Math.PI * 8;
  /**
   * 
   */
  private final double MAX_SPEED = 1000; //need to define
  /**
   * Timeout constant for PIDF control in milliseconds
   */
  private final int kTimeoutMs = 30;
  /**
   * PIDF constants for velocity control
   */
  private final double KF = 0.0, KP = 0.0, KI = 0.0, KD = 0.0;
  /**
   * Deadband constant used to enable quickturning
   */
  public final double DEADBAND = 0.05;

  /**
   * Instantiates the talons and accelerometer and configures PIDF control
   */
  public VelocityDrive() {
    lFront = new WPI_TalonSRX(RobotMap.LEFT_TALON_FRONT);
    rFront = new WPI_TalonSRX(RobotMap.RIGHT_TALON_FRONT);
    lBack = new WPI_TalonSRX(RobotMap.LEFT_TALON_BACK);
    rBack = new WPI_TalonSRX(RobotMap.RIGHT_TALON_BACK);

    try {
      ahrs = new AHRS(Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    lBack.set(ControlMode.Follower, lFront.getDeviceID());
    rBack.set(ControlMode.Follower, rFront.getDeviceID());

    lBack.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rBack.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    setBrakeMode(true);

    lBack.config_kF(0, KF, kTimeoutMs);
    lBack.config_kP(0, KP, kTimeoutMs);
    lBack.config_kI(0, KI, kTimeoutMs);
    lBack.config_kD(0, KD, kTimeoutMs);

    rBack.config_kF(0, KF, kTimeoutMs);
    rBack.config_kP(0, KP, kTimeoutMs);
    rBack.config_kI(0, KI, kTimeoutMs);
    rBack.config_kD(0, KD, kTimeoutMs);
  }

  /**
   * set each the motors' speeds (need to determine the unit to use)
   * @param left left motors's speedn (need to determine the unit to use)
   * @param right
   */
  public void setMotors(double left, double right) {
    lFront.set(ControlMode.Velocity, left);
    rFront.set(ControlMode.Velocity, right);
  }

  /**
   * Set the motors based on a drivesignal
   * @param signal the drivesignal to set the motors to
   */
  public void drive(DriveSignal signal) {
    setMotors(signal.getLeft()*MAX_SPEED, signal.getRight()*MAX_SPEED);
  }

  /**
   * zero out Yaw on the accelerometer
   */
  public void zeroYaw() {
    ahrs.zeroYaw();
  }

  /**
   * get the yaw for the accelerometer
   * @return the yaw for the accelerometer (need to find out unit)
   */
  public double getYaw() {
    return ahrs.getYaw();
  }
  
  /**
   * set the brake mode for the motors
   * @param brake true to turn on breaking, false will have the motors coast
   */
  public void setBrakeMode(boolean brake) {
    if (brake) {
      lFront.setNeutralMode(NeutralMode.Brake);
      rFront.setNeutralMode(NeutralMode.Brake);
    } else {
      lFront.setNeutralMode(NeutralMode.Coast);
      rFront.setNeutralMode(NeutralMode.Coast);
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}