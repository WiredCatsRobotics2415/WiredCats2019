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
   * shuffleboard values
   */
  private NetworkTableEntry networkTableDistanceKP, networkTableDistanceKI, networkTableDistanceKD, networkTableDistanceKF, leftDistancePIDError, rightDistancePIDError, leftVelocity,rightVelocity;
  private NetworkTableEntry networkTableTurnKP, networkTableTurnKI, networkTableTurnKD, networkTableTurnKF, leftTurnPIDError, rightTurnPIDError, turnSpeed;
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
  private final double MAX_TURN = 1000; //TODO
  /**
   * Timeout constant for PIDF control in milliseconds
   */
  private final int kTimeoutMs = 30;
  /**
   * PIDF constants for velocity control
   */
  private final double DISTANCE_KF = 1023/MAX_SPEED, DISTANCE_KP = 0.0, DISTANCE_KI = 0.0, DISTANCE_KD = 0.0, DISTANCE_MAX_OUTPUT = 1.0;
  private final int PRIMARY_PID_INX = 0;
  private final double TURN_KF = 0.0, TURN_KP = 0.0, TURN_KI = 0.0, TURN_KD = 0.0, TURN_MAX_OUTPUT = 1.0;
  private final int TURN_PID_INX = 1;
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

    lBack.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PRIMARY_PID_INX, kTimeoutMs);
    rBack.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PRIMARY_PID_INX, kTimeoutMs);

    setBrakeMode(true);

    setDistancePIDFConstants(DISTANCE_KP, DISTANCE_KI, DISTANCE_KD, DISTANCE_KF);
    setTurnPIDFConstants(TURN_KP, TURN_KI, TURN_KD, TURN_KF);
    lBack.configAuxPIDPolarity(true, kTimeoutMs);
    rBack.configAuxPIDPolarity(false, kTimeoutMs);
    lBack.configClosedLoopPeakOutput(PRIMARY_PID_INX, DISTANCE_MAX_OUTPUT);
    rBack.configClosedLoopPeakOutput(PRIMARY_PID_INX, DISTANCE_MAX_OUTPUT);
    lBack.configClosedLoopPeakOutput(TURN_PID_INX, TURN_MAX_OUTPUT);
    rBack.configClosedLoopPeakOutput(TURN_PID_INX, TURN_MAX_OUTPUT);
    this.PIDFTuning = PIDFTuning;
    if(PIDFTuning) {
      manualPIDFTuner = Shuffleboard.getTab("PIDF Tuner");
      networkTableDistanceKP = manualPIDFTuner.add("Distance KP", DISTANCE_KP).getEntry();
      networkTableDistanceKI = manualPIDFTuner.add("Distance KI", DISTANCE_KI).getEntry();
      networkTableDistanceKD = manualPIDFTuner.add("Distance KD", DISTANCE_KD).getEntry();
      networkTableDistanceKF = manualPIDFTuner.add("Distance KF", DISTANCE_KF).getEntry();
      leftDistancePIDError = manualPIDFTuner.add("Left Distance PID Error", 0.0).getEntry();
      rightDistancePIDError = manualPIDFTuner.add("Right Distance PID Error", 0.0).getEntry();
      leftVelocity = manualPIDFTuner.add("Left Velocity", 0.0).getEntry();
      rightVelocity = manualPIDFTuner.add("Right Velocity", 0.0).getEntry();
    } else {
      manualPIDFTuner = null;
      networkTableDistanceKP = null;
      networkTableDistanceKI = null;
      networkTableDistanceKD = null;
      networkTableDistanceKF = null;
      leftDistancePIDError = null;
      rightDistancePIDError = null;
      leftVelocity = null;
      rightVelocity = null;
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
      leftDistancePIDError.setDouble((double)lBack.getClosedLoopError());
      rightDistancePIDError.setDouble((double)rBack.getClosedLoopError());
      leftVelocity.setDouble((double)lBack.getSelectedSensorVelocity());
      rightVelocity.setDouble((double)rBack.getSelectedSensorVelocity());
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
    double KP = DISTANCE_KP, KI = DISTANCE_KI, KD = DISTANCE_KD, KF = DISTANCE_KF;
    if(PIDFTuning) {
      KP = networkTableDistanceKP.getDouble(KP);
      KI = networkTableDistanceKI.getDouble(KI);
      KD = networkTableDistanceKD.getDouble(KD);
      KF = networkTableDistanceKF.getDouble(KF);
      
      setDistancePIDFConstants(KP, KI, KD, KF);
    }
  }

  /**
   * Set the parameters for PID control
   * @param KP P parameter
   * @param KI I parameter
   * @param KD D parameter
   * @param DF F parameter
   */
  public void setDistancePIDFConstants(double KP, double KI, double KD, double KF) {
    lBack.config_kF(PRIMARY_PID_INX, KF, kTimeoutMs);
    lBack.config_kP(PRIMARY_PID_INX, KP, kTimeoutMs);
    lBack.config_kI(PRIMARY_PID_INX, KI, kTimeoutMs);
    lBack.config_kD(PRIMARY_PID_INX, KD, kTimeoutMs);

    rBack.config_kF(PRIMARY_PID_INX, KF, kTimeoutMs);
    rBack.config_kP(PRIMARY_PID_INX, KP, kTimeoutMs);
    rBack.config_kI(PRIMARY_PID_INX, KI, kTimeoutMs);
    rBack.config_kD(PRIMARY_PID_INX, KD, kTimeoutMs);
  }

  public void setTurnPIDFConstants(double KP, double KI, double KD, double KF) {
    lBack.config_kF(TURN_PID_INX, KF, kTimeoutMs);
    lBack.config_kP(TURN_PID_INX, KP, kTimeoutMs);
    lBack.config_kI(TURN_PID_INX, KI, kTimeoutMs);
    lBack.config_kD(TURN_PID_INX, KD, kTimeoutMs);

    rBack.config_kF(TURN_PID_INX, KF, kTimeoutMs);
    rBack.config_kP(TURN_PID_INX, KP, kTimeoutMs);
    rBack.config_kI(TURN_PID_INX, KI, kTimeoutMs);
    rBack.config_kD(TURN_PID_INX, KD, kTimeoutMs);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}