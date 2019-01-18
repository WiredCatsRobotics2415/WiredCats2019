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
 * Add your docs here.
 */
public class VelocityDrive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX lFront, rFront, lBack, rBack;
  public AHRS ahrs;

  private final double WHEEL_CIRCUMFERENCE = Math.PI * 8; //inches
  private final double MAX_SPEED = 1000; //need to define
  
  private final int kTimeoutMs = 30;
  private final double KF = 0.0; //need to define
  private final double KP = 0.0; //need to define
  private final double KI = 0.0; //need to define
  private final double KD = 0.0; //need to define

  public final double DEADBAND = 0.05;

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

  private void setMotors(double left, double right) {
    lFront.set(ControlMode.Velocity, left);
    rFront.set(ControlMode.Velocity, right);
  }

  public void drive(DriveSignal signal) {
    setMotors(signal.getLeft()*MAX_SPEED, signal.getRight()*MAX_SPEED);
  }

  public void zeroYaw() {
    ahrs.zeroYaw();
  }

  public double getYaw() {
    return ahrs.getYaw();
  }
  
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
