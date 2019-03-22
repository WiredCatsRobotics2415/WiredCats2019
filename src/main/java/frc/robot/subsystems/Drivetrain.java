/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.RobotMap;
import frc.robot.cheesy.DriveSignal;
import frc.robot.util.pid.PIDTunable;
import frc.robot.util.pid.PIDValue;
import frc.robot.Constants;

public class Drivetrain extends Subsystem implements PIDTunable{
  //motors and sensors
  private WPI_TalonSRX lFront, rFront, lBack, rBack;
  public AHRS ahrs;

  //class varibles
  private WPI_TalonSRX lMaster, rMaster;
  private Drivemode drivemode;
  private PIDValue[] pidValues;
  
  public double DEADBAND = 0.05;

  public Drivetrain() {
    this(Drivemode.percentOutput);
  }

  public Drivetrain(Drivemode drivemode) {
    lFront = new WPI_TalonSRX(RobotMap.LEFT_TALON_FRONT);
    lFront.configFactoryDefault();
    rFront = new WPI_TalonSRX(RobotMap.RIGHT_TALON_FRONT);
    rFront.configFactoryDefault();
    lBack = new WPI_TalonSRX(RobotMap.LEFT_TALON_BACK);
    lBack.configFactoryDefault();
    rBack = new WPI_TalonSRX(RobotMap.RIGHT_TALON_BACK);
    rBack.configFactoryDefault();
    // TalonSRXConfiguration test = new TalonSRXConfiguration();
    // lFront.getAllConfigs(test);
    // System.out.println("hi");
    // System.out.println(test);

    lFront.setInverted(RobotMap.LEFT_TALON_FRONT_DIRECTION);
    lBack.setInverted(RobotMap.LEFT_TALON_BACK_DIRECTION);
    rFront.setInverted(RobotMap.RIGHT_TALON_FRONT_DIRECTION);
    rBack.setInverted(RobotMap.RIGHT_TALON_BACK_DIRECTION);

    // lFront.follow(lBack);
    // rFront.follow(rBack);
    lMaster = lBack;
    rMaster = rBack;

    setBrakeMode(true);

    lMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    //lMaster.configOpenloopRamp(0);
    //rMaster.configOpenloopRamp(0);

    setDrivemode(drivemode);
    // try {
    //   ahrs = new AHRS(Port.kMXP);
    // } catch (RuntimeException ex) {
    //   ahrs = null;
    // }
  }

  public void babyDrive(double throttle, double turn) {
    lBack.set(ControlMode.PercentOutput, throttle+turn);
    lFront.set(ControlMode.PercentOutput, throttle+turn);
    rBack.set(ControlMode.PercentOutput, throttle-turn);
    rFront.set(ControlMode.PercentOutput, throttle-turn);
  }

  public void enableSafeties() {
    lMaster.setSafetyEnabled(true);
    rMaster.setSafetyEnabled(true);
  }

  public void disableSafeties() {
    lMaster.setSafetyEnabled(false);
    rMaster.setSafetyEnabled(false);
  }
   

  public void setMotors(double left, double right) {
    switch(drivemode) {
      case percentOutput:
        lMaster.set(ControlMode.PercentOutput, left);
        rMaster.set(ControlMode.PercentOutput, right);
        break;
      case velocity:
        lMaster.set(ControlMode.Velocity, left*Constants.DRIVETRAIN_MAX_SPEED);
        rMaster.set(ControlMode.Velocity, left*Constants.DRIVETRAIN_MAX_SPEED);
        break;
      default:
        lMaster.set(ControlMode.PercentOutput, left);
        rMaster.set(ControlMode.PercentOutput, right);
        break;
    }
  }

  public void drive(double throttle, double turn) {
    boolean isQuickTurn;
    if (Math.abs(throttle) < Math.abs(Constants.DEADBAND)) throttle = 0;
    if (Math.abs(turn) < Math.abs(Constants.DEADBAND)) turn = 0;
    setMotors(throttle+turn,throttle-turn);
  }

  public void drive(DriveSignal driveSignal) {
    setMotors(driveSignal.getLeft(),driveSignal.getRight());
  }

  public void setBrakeMode(boolean brake) {
    if(brake) {
      lMaster.setNeutralMode(NeutralMode.Brake);
      rMaster.setNeutralMode(NeutralMode.Brake);
    } else {
      lMaster.setNeutralMode(NeutralMode.Coast);
      rMaster.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void setDrivemode(Drivemode drivemode) {
    this.drivemode = drivemode;
    switch(this.drivemode) {
      case percentOutput:
        pidValues = new PIDValue[0];
        break;
      case velocity:
        pidValues = new PIDValue[1];
        pidValues[0] = new PIDValue(Constants.VELOCITY_PID);
        lMaster.config_kP(Constants.VELOCITY_PID_INDEX, Constants.VELOCITY_PID.getKP());
        lMaster.config_kP(Constants.VELOCITY_PID_INDEX, Constants.VELOCITY_PID.getKI());
        lMaster.config_kP(Constants.VELOCITY_PID_INDEX, Constants.VELOCITY_PID.getKD());
        lMaster.config_kP(Constants.VELOCITY_PID_INDEX, Constants.VELOCITY_PID.getKF());
  
        rMaster.config_kP(Constants.VELOCITY_PID_INDEX, Constants.VELOCITY_PID.getKP());
        rMaster.config_kP(Constants.VELOCITY_PID_INDEX, Constants.VELOCITY_PID.getKI());
        rMaster.config_kP(Constants.VELOCITY_PID_INDEX, Constants.VELOCITY_PID.getKD());
        rMaster.config_kP(Constants.VELOCITY_PID_INDEX, Constants.VELOCITY_PID.getKF());
        break;
      default:
        pidValues = new PIDValue[0];
        break;
    }
  }

  @Override
  public void setPIDFConstants(PIDValue[] newValues) {
    switch(drivemode) {
      case percentOutput:
        break;
      case velocity:
        if(newValues.length > 0) {
          pidValues[0] = new PIDValue(newValues[0]);
          lMaster.config_kP(Constants.VELOCITY_PID_INDEX, pidValues[0].getKP());
          lMaster.config_kP(Constants.VELOCITY_PID_INDEX, pidValues[0].getKI());
          lMaster.config_kP(Constants.VELOCITY_PID_INDEX, pidValues[0].getKD());
          lMaster.config_kP(Constants.VELOCITY_PID_INDEX, pidValues[0].getKF());
  
          rMaster.config_kP(Constants.VELOCITY_PID_INDEX, pidValues[0].getKP());
          rMaster.config_kP(Constants.VELOCITY_PID_INDEX, pidValues[0].getKI());
          rMaster.config_kP(Constants.VELOCITY_PID_INDEX, pidValues[0].getKD());
          rMaster.config_kP(Constants.VELOCITY_PID_INDEX, pidValues[0].getKF());
        }
        break;
      default:
        break;
    }
  }

  @Override
  public double[] getPIDOutput() {
    double[] output;
    switch(drivemode) {
      case percentOutput:
        output = new double[0];
        return output;
      case velocity:
        output = new double[1];
        output[0] = (lMaster.getSelectedSensorVelocity()+rMaster.getSelectedSensorVelocity())/2.0;
        return output;
      default:
        output = new double[0];
        return output;
    }
  }

  public double[] getVelocity() {
    double[] velocities = new double[2];
    velocities[0] = lBack.getSelectedSensorVelocity();
    velocities[1] = rBack.getSelectedSensorVelocity();
    return velocities;
  }

  public void zeroYaw() {
    ahrs.zeroYaw();
  }

  public double getYaw() {
    return ahrs.getYaw();
  }

  public double getBusVoltage() {
    return lBack.getBusVoltage();
  }

  public void printCurrent() {
    System.out.println("lFront current="+lFront.getOutputCurrent());
    System.out.println("lBack current="+lBack.getOutputCurrent());
    System.out.println("rFront current="+rFront.getOutputCurrent());
    System.out.println("rBack current="+rBack.getOutputCurrent());
    System.out.println();
  }

  public void testMotor(double speed) {
    // rFront.set(speed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  enum Drivemode {
    percentOutput, velocity;
  }
}
