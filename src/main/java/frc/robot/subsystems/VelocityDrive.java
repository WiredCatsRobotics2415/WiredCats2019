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

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
   * shuffleboard tab for PID tuning
   */
  private ShuffleboardTab manualPIDFTuner;
  /**
   * shuffleboard value for P for PID tuning
   */
  private NetworkTableEntry networkTableKP;
  /**
   * shuffleboard value for I for PID tuning
   */
  private NetworkTableEntry networkTableKI;
  /**
   * shuffleboard value for D for PID tuning
   */
  private NetworkTableEntry networkTableKD;
  /**
   * shuffleboard value for F for PID tuning
   */
  private NetworkTableEntry networkTableKF;
  /**
   * shuffleboard value the error of the left motors
   */
  private NetworkTableEntry leftPIDError;
  /**
   * shuffleboard value the error of the right motors
   */
  private NetworkTableEntry rightPIDError;
  /**
   * if pid tuning is being done
   */
  private boolean PIDFTuning;
  /**
   * A constant to hold the cirumference of the wheel in inches for distance calculations
   */
  private final double WHEEL_CIRCUMFERENCE = Math.PI * 8;
  /**
   * Encoder tick per rotation
   */
  private final double ENCODER_TICK_ROTATION = 4096;
  /**
   * ticks per 100ms;
   */
  private final double MAX_SPEED = 3628.0;
  /**
   * Timeout constant for PIDF control in milliseconds
   */
  private final int kTimeoutMs = 30;
  /**
   * PIDF constants for velocity control
   */
  private final double KF = 1023/MAX_SPEED, KP = 0.0, KI = 0.0, KD = 0.0;
  /**
   * Deadband constant used to enable quickturning
   */
  public final double DEADBAND = 0.05;

  /**
   * Instantiates the talons and accelerometer and configures PIDF control and sets PIDF tuning to false
   */
  public VelocityDrive() {
    this(false);
  }

  /**
   * Instantiates the talons and accelerometer and configures PIDF control and sets PIDF tuning to the parameter
   * @param PIDFTuning if the PIDF varibles are being tuned from shuffleboard
   */
  public VelocityDrive(boolean PIDFTuning) {
    lFront = new WPI_TalonSRX(RobotMap.LEFT_TALON_FRONT);
    rFront = new WPI_TalonSRX(RobotMap.RIGHT_TALON_FRONT);
    lBack = new WPI_TalonSRX(RobotMap.LEFT_TALON_BACK);
    rBack = new WPI_TalonSRX(RobotMap.RIGHT_TALON_BACK);

    try {
      //ahrs = new AHRS(Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    lFront.setInverted(true);
    lBack.setInverted(true);
    rFront.setInverted(false);
    rBack.setInverted(false);

    lFront.set(ControlMode.Follower, lFront.getDeviceID());
    rFront.set(ControlMode.Follower, rFront.getDeviceID());

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
    this.PIDFTuning = PIDFTuning;
    if(PIDFTuning) {
      manualPIDFTuner = Shuffleboard.getTab("PIDF Tuner");
      networkTableKP = manualPIDFTuner.add("KP", KP).getEntry();
      networkTableKI = manualPIDFTuner.add("KI", KI).getEntry();
      networkTableKD = manualPIDFTuner.add("KD", KD).getEntry();
      networkTableKF = manualPIDFTuner.add("KF", KF).getEntry();
      leftPIDError = manualPIDFTuner.add("Left PID Error", 0.0).getEntry();
      rightPIDError = manualPIDFTuner.add("Right PID Error", 0.0).getEntry();
    } else {
      manualPIDFTuner = null;
      networkTableKP = null;
      networkTableKI = null;
      networkTableKD = null;
      networkTableKF = null;
      leftPIDError = null;
      rightPIDError = null;
    }
  }

  /**
   * set each the motors' speeds (need to determine the unit to use)
   * @param left left motors's speedn (need to determine the unit to use)
   * @param right right motors's speedn (need to determine the unit to use)
   */
  public void setMotors(double left, double right) {
    lBack.set(ControlMode.Velocity, left);
    rBack.set(ControlMode.Velocity, right);
  }

  /**
   * Set the motors based on a drivesignal
   * @param signal the drivesignal to set the motors to
   */
  public void drive(DriveSignal signal) {
    if(PIDFTuning) {
      refreshPIDValuesNetworkTable();
      leftPIDError.setDouble((double)lBack.getClosedLoopError());
      rightPIDError.setDouble((double)rBack.getClosedLoopError());
    }
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

  /**
   * refreshes PID parameters from shuffleboard
   */
  public void refreshPIDValuesNetworkTable() {
    double KP = this.KP, KI = this.KI, KD = this.KD, KF = this.KF;
    if(PIDFTuning) {
      KP = networkTableKP.getDouble(KP);
      KI = networkTableKI.getDouble(KI);
      KD = networkTableKD.getDouble(KD);
      KF = networkTableKF.getDouble(KF);
      
      setPIDFConstants(KP, KI, KD, KF);
    }
  }

  /**
   * Set the parameters for PID control
   * @param KP P parameter
   * @param KI I parameter
   * @param KD D parameter
   * @param DF F parameter
   */
  public void setPIDFConstants(double KP, double KI, double KD, double KF) {
    lBack.config_kF(0, KF, kTimeoutMs);
    lBack.config_kP(0, KP, kTimeoutMs);
    lBack.config_kI(0, KI, kTimeoutMs);
    lBack.config_kD(0, KD, kTimeoutMs);

    rBack.config_kF(0, KF, kTimeoutMs);
    rBack.config_kP(0, KP, kTimeoutMs);
    rBack.config_kI(0, KI, kTimeoutMs);
    rBack.config_kD(0, KD, kTimeoutMs);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}