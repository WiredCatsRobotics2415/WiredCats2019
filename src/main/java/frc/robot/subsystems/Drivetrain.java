/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;

import org.junit.Test.None;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.RobotMap;
import frc.robot.cheesy.DriveSignal;
import frc.robot.util.pid.PIDTunable;
import frc.robot.util.pid.PIDValue;
import frc.robot.Constants;
import frc.robot.Robot;

public class Drivetrain extends Subsystem implements PIDTunable, PIDSource, PIDOutput{
  //motors and sensors
  private WPI_TalonSRX lFront, rFront, lBack, rBack;
  public PigeonIMU pidgey;
  public AHRS ahrs;

  //class varibles
  private WPI_TalonSRX lMaster, rMaster;
  private Drivemode drivemode;
  private PIDValue[] pidValues;
  private boolean usingPidgey;
  private PIDController pidController;
  private PIDSourceType pidSourceType;
  private double previousThrottle;

  public Drivetrain() {
    this(Drivemode.percentOutput);
  }

  public Drivetrain(Drivemode drivemode) {
    lFront = Robot.getTalon(RobotMap.LEFT_TALON_FRONT);
    rFront = Robot.getTalon(RobotMap.RIGHT_TALON_FRONT);
    lBack = Robot.getTalon(RobotMap.LEFT_TALON_BACK);
    rBack = Robot.getTalon(RobotMap.RIGHT_TALON_BACK);

    lFront.setInverted(RobotMap.LEFT_TALON_FRONT_DIRECTION);
    lBack.setInverted(RobotMap.LEFT_TALON_BACK_DIRECTION);
    rFront.setInverted(RobotMap.RIGHT_TALON_FRONT_DIRECTION);
    rBack.setInverted(RobotMap.RIGHT_TALON_BACK_DIRECTION);

    lFront.follow(lBack);
    rFront.follow(rBack);
    lMaster = lBack;
    rMaster = rBack;

    setBrakeMode(true);

    lMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
    lMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
    rMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
    rMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

    if(lMaster.getDeviceID() == RobotMap.LEFT_DRIVETRAIN_ENCODER) {
      lMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.VELOCITY_PID_INDEX, Constants.kTimeoutMs);
    } else {
      System.out.println("wrong port on Left Encoder");
    }
    if(rMaster.getDeviceID() == RobotMap.RIGHT_DRIVETRAIN_ENCODER) {
      rMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.VELOCITY_PID_INDEX, Constants.kTimeoutMs);
    } else {
      System.out.println("wrong port on Right Encoder");
    }

    zeroEncoders();
    
    if(RobotMap.PIGEON_ID >= 0) {
      if(RobotMap.PIGEON_ON_CAN) {
        pidgey = new PigeonIMU(RobotMap.PIGEON_ID);
      } else {
        pidgey = new PigeonIMU(Robot.getTalon(RobotMap.PIGEON_ID));
      }
    }

    try {
      ahrs = new AHRS(Port.kMXP);
    } catch (RuntimeException ex) {
      ahrs = null;
    }

    if(Constants.PIGEON_DEFAULT && pidgey != null) {
      usingPidgey = true;
    } else {
      usingPidgey = false;
    }

    zeroGyro();

    if(usingPidgey) {
      if(RobotMap.PIGEON_ON_CAN) {
        switch(RobotMap.PIGEON_DIRECTION) {
          case Yaw:
            lMaster.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.Pigeon_Yaw, Constants.PIGEON_REMOTE_INDEX);
            rMaster.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.Pigeon_Yaw, Constants.PIGEON_REMOTE_INDEX);
            break;
          case Pitch:
            lMaster.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.Pigeon_Pitch, Constants.PIGEON_REMOTE_INDEX);
            rMaster.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.Pigeon_Pitch, Constants.PIGEON_REMOTE_INDEX);
            break;
          case Roll:
            lMaster.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.Pigeon_Roll, Constants.PIGEON_REMOTE_INDEX);
            rMaster.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.Pigeon_Roll, Constants.PIGEON_REMOTE_INDEX);
            break;
          default:
            lMaster.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.Pigeon_Yaw, Constants.PIGEON_REMOTE_INDEX);
            rMaster.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.Pigeon_Yaw, Constants.PIGEON_REMOTE_INDEX);
            break;
        }
      } else {
        lMaster.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Yaw, Constants.PIGEON_REMOTE_INDEX);
        rMaster.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Yaw, Constants.PIGEON_REMOTE_INDEX);
        switch(RobotMap.PIGEON_DIRECTION) {
          case Yaw:
            lMaster.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Yaw, Constants.PIGEON_REMOTE_INDEX);
            rMaster.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Yaw, Constants.PIGEON_REMOTE_INDEX);
            break;
          case Pitch:
            lMaster.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Pitch, Constants.PIGEON_REMOTE_INDEX);
            rMaster.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Pitch, Constants.PIGEON_REMOTE_INDEX);
            break;
          case Roll:
            lMaster.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Roll, Constants.PIGEON_REMOTE_INDEX);
            rMaster.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Roll, Constants.PIGEON_REMOTE_INDEX);
            break;
          default:
            lMaster.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Yaw, Constants.PIGEON_REMOTE_INDEX);
            rMaster.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Yaw, Constants.PIGEON_REMOTE_INDEX);
            break;
        }
      }
      if(Constants.PIGEON_REMOTE_INDEX == 0) {
        lMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, Constants.TURN_PID_INDEX, Constants.kTimeoutMs);
        rMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, Constants.TURN_PID_INDEX, Constants.kTimeoutMs);
      } else {
        lMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, Constants.TURN_PID_INDEX, Constants.kTimeoutMs);
        rMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, Constants.TURN_PID_INDEX, Constants.kTimeoutMs);
      }
      lMaster.configSelectedFeedbackCoefficient(Constants.PIGEON_UNITS2DEGREES);
    }
    
    pidController = new PIDController(0, 0, 0, 0, this, this, 0.02);
    pidSourceType = PIDSourceType.kDisplacement;
    previousThrottle = 0.0;
    setDrivemode(drivemode);
  }
   

  public void setMotors(double throttle, double turn) {
    switch(drivemode) {
      case percentOutput:
        lMaster.set(ControlMode.PercentOutput, throttle+turn);
        rMaster.set(ControlMode.PercentOutput, throttle-turn);
        break;
      case velocity:
        lMaster.set(ControlMode.Velocity, (throttle+turn)*Constants.DRIVETRAIN_MAX_SPEED);
        rMaster.set(ControlMode.Velocity, (throttle-turn)*Constants.DRIVETRAIN_MAX_SPEED);
        break;
      case percentOutputTurnControl:
        if(usingPidgey) {
          lMaster.set(ControlMode.PercentOutput, throttle, DemandType.AuxPID, turn);
          rMaster.set(ControlMode.PercentOutput, throttle, DemandType.AuxPID, turn);
        } else {
          lMaster.set(ControlMode.PercentOutput, throttle+turn);
          rMaster.set(ControlMode.PercentOutput, throttle-turn);
        }
      default:
        lMaster.set(ControlMode.PercentOutput, throttle+turn);
        rMaster.set(ControlMode.PercentOutput, throttle-turn);
        break;
    }
  }

  public void setMotorsInd(double left, double right) {
    switch(drivemode) {
      case percentOutput:
        lMaster.set(ControlMode.PercentOutput, left);
        rMaster.set(ControlMode.PercentOutput, right);
        break;
      case velocity:
        lMaster.set(ControlMode.Velocity, (left)*Constants.DRIVETRAIN_MAX_SPEED);
        rMaster.set(ControlMode.Velocity, (right)*Constants.DRIVETRAIN_MAX_SPEED);
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
    if(drivemode == Drivemode.percentOutputTurnControl && !usingPidgey) {
      pidController.setSetpoint(turn);
      previousThrottle = throttle;
      return;
    }
    setMotors(throttle,turn);
  }

  public void drive(DriveSignal driveSignal) {
    setMotorsInd(driveSignal.getLeft(),driveSignal.getRight());
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

  public void setRampOpenLoop(double timeToFull) {
    lMaster.configOpenloopRamp(timeToFull);
    rMaster.configOpenloopRamp(timeToFull);
  }

  public void setDrivemode(Drivemode drivemode) {
    this.drivemode = drivemode;
    pidController.disable();
    switch(this.drivemode) {
      case percentOutput:
        pidValues = new PIDValue[0];
        setRampOpenLoop(Constants.STANDARD_RAMP);
        break;
      case velocity:
        setRampOpenLoop(Constants.STANDARD_RAMP);
        pidValues = new PIDValue[1];
        pidValues[0] = new PIDValue(Constants.VELOCITY_PID);
        lMaster.config_kP(Constants.VELOCITY_PID_INDEX, Constants.VELOCITY_PID.getKP(), Constants.kTimeoutMs);
        lMaster.config_kP(Constants.VELOCITY_PID_INDEX, Constants.VELOCITY_PID.getKI(), Constants.kTimeoutMs);
        lMaster.config_kP(Constants.VELOCITY_PID_INDEX, Constants.VELOCITY_PID.getKD(), Constants.kTimeoutMs);
        lMaster.config_kP(Constants.VELOCITY_PID_INDEX, Constants.VELOCITY_PID.getKF(), Constants.kTimeoutMs);
  
        rMaster.config_kP(Constants.VELOCITY_PID_INDEX, Constants.VELOCITY_PID.getKP(), Constants.kTimeoutMs);
        rMaster.config_kP(Constants.VELOCITY_PID_INDEX, Constants.VELOCITY_PID.getKI(), Constants.kTimeoutMs);
        rMaster.config_kP(Constants.VELOCITY_PID_INDEX, Constants.VELOCITY_PID.getKD(), Constants.kTimeoutMs);
        rMaster.config_kP(Constants.VELOCITY_PID_INDEX, Constants.VELOCITY_PID.getKF(), Constants.kTimeoutMs);
        break;
      case percentOutputTurnControl:
        setRampOpenLoop(Constants.STANDARD_RAMP);
        pidValues = new PIDValue[1];
        pidValues[0] = new PIDValue(Constants.TURN_PID);
        if(usingPidgey) {
          lMaster.config_kP(Constants.TURN_PID_INDEX, Constants.TURN_PID.getKP(), Constants.kTimeoutMs);
          lMaster.config_kP(Constants.TURN_PID_INDEX, Constants.TURN_PID.getKI(), Constants.kTimeoutMs);
          lMaster.config_kP(Constants.TURN_PID_INDEX, Constants.TURN_PID.getKD(), Constants.kTimeoutMs);
          lMaster.config_kP(Constants.TURN_PID_INDEX, Constants.TURN_PID.getKF(), Constants.kTimeoutMs);
  
          rMaster.config_kP(Constants.TURN_PID_INDEX, Constants.TURN_PID.getKP(), Constants.kTimeoutMs);
          rMaster.config_kP(Constants.TURN_PID_INDEX, Constants.TURN_PID.getKI(), Constants.kTimeoutMs);
          rMaster.config_kP(Constants.TURN_PID_INDEX, Constants.TURN_PID.getKD(), Constants.kTimeoutMs);
          rMaster.config_kP(Constants.TURN_PID_INDEX, Constants.TURN_PID.getKF(), Constants.kTimeoutMs);

          lMaster.configAuxPIDPolarity(false, Constants.kTimeoutMs);
          rMaster.configAuxPIDPolarity(true, Constants.kTimeoutMs);
        } else {
          pidController.setPID(Constants.TURN_PID.getKP(), Constants.TURN_PID.getKI(), Constants.TURN_PID.getKD(), Constants.TURN_PID.getKF());
          pidController.setOutputRange(-1, 1);
          pidController.enable();
        }
      default:
        setRampOpenLoop(Constants.STANDARD_RAMP);
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
          lMaster.config_kP(Constants.VELOCITY_PID_INDEX, pidValues[0].getKP(), Constants.kTimeoutMs);
          lMaster.config_kP(Constants.VELOCITY_PID_INDEX, pidValues[0].getKI(), Constants.kTimeoutMs);
          lMaster.config_kP(Constants.VELOCITY_PID_INDEX, pidValues[0].getKD(), Constants.kTimeoutMs);
          lMaster.config_kP(Constants.VELOCITY_PID_INDEX, pidValues[0].getKF(), Constants.kTimeoutMs);
  
          rMaster.config_kP(Constants.VELOCITY_PID_INDEX, pidValues[0].getKP(), Constants.kTimeoutMs);
          rMaster.config_kP(Constants.VELOCITY_PID_INDEX, pidValues[0].getKI(), Constants.kTimeoutMs);
          rMaster.config_kP(Constants.VELOCITY_PID_INDEX, pidValues[0].getKD(), Constants.kTimeoutMs);
          rMaster.config_kP(Constants.VELOCITY_PID_INDEX, pidValues[0].getKF(), Constants.kTimeoutMs);
        }
        break;
      case percentOutputTurnControl:
        if(newValues.length > 0) {
          pidValues[0] = new PIDValue(newValues[0]);
          if(usingPidgey) {
            lMaster.config_kP(Constants.TURN_PID_INDEX, pidValues[0].getKP(), Constants.kTimeoutMs);
            lMaster.config_kP(Constants.TURN_PID_INDEX, pidValues[0].getKI(), Constants.kTimeoutMs);
            lMaster.config_kP(Constants.TURN_PID_INDEX, pidValues[0].getKD(), Constants.kTimeoutMs);
            lMaster.config_kP(Constants.TURN_PID_INDEX, pidValues[0].getKF(), Constants.kTimeoutMs);
  
            rMaster.config_kP(Constants.TURN_PID_INDEX, pidValues[0].getKP(), Constants.kTimeoutMs);
            rMaster.config_kP(Constants.TURN_PID_INDEX, pidValues[0].getKI(), Constants.kTimeoutMs);
            rMaster.config_kP(Constants.TURN_PID_INDEX, pidValues[0].getKD(), Constants.kTimeoutMs);
            rMaster.config_kP(Constants.TURN_PID_INDEX, pidValues[0].getKF(), Constants.kTimeoutMs);
          } else {
            pidController.setPID(pidValues[0].getKP(), pidValues[0].getKI(), pidValues[0].getKD(), pidValues[0].getKF());
          }
        }
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
      case percentOutputTurnControl:
        output = new double[1];
        output[0] = getGryo();
      default:
        output = new double[0];
        return output;
    }
  }

  @Override
  public void pidWrite(double output) {
    if(drivemode == Drivemode.percentOutputTurnControl && !usingPidgey) {
      setMotors(previousThrottle, output);
    }
  }

  @Override
  public double pidGet() {
    if(drivemode == Drivemode.percentOutputTurnControl) {
      return getGryo();
    }
    return 0;
  }

  @Override
  public PIDSourceType getPIDSourceType() {
    return pidSourceType;
  }

  @Override
  public void setPIDSourceType(PIDSourceType sourceType) {
    this.pidSourceType = sourceType;
  }

  public double[] getVelocity() { //native unit/100 ms
    double[] velocities = new double[2];
    velocities[0] = lBack.getSelectedSensorVelocity();
    velocities[1] = rBack.getSelectedSensorVelocity();
    return velocities;
  }

  public void zeroGyro() {
    if(ahrs != null) {
      ahrs.zeroYaw();
    }
    if(pidgey != null) {
      pidgey.setYaw(0, Constants.kTimeoutMs);
    }
  }

  public double getGryo() {
    if(Constants.PIGEON_DEFAULT && pidgey != null) {
      double[] ypr = new double[3];
      pidgey.getYawPitchRoll(ypr);
      switch(RobotMap.PIGEON_DIRECTION) {
        case Yaw:
          return ypr[0]*Constants.PIGEON_UNITS2DEGREES;
        case Pitch:
          return ypr[1]*Constants.PIGEON_UNITS2DEGREES;
        case Roll:
          return ypr[2]*Constants.PIGEON_UNITS2DEGREES;
        default:
          return ypr[0]*Constants.PIGEON_UNITS2DEGREES;
      }
    }
    if(ahrs != null) {
      switch(RobotMap.PIGEON_DIRECTION) {
        case Yaw:
          return (double)ahrs.getYaw()*Constants.NAVX_UNITS2DEGREES;
        case Pitch:
          return (double)ahrs.getPitch()*Constants.NAVX_UNITS2DEGREES;
        case Roll:
          return (double)ahrs.getRoll()*Constants.NAVX_UNITS2DEGREES;
        default:
          return (double)ahrs.getYaw()*Constants.NAVX_UNITS2DEGREES;
      }
    }
    return 0;
  }

  public void zeroEncoders() {
    lMaster.setSelectedSensorPosition(0, Constants.VELOCITY_PID_INDEX, Constants.kTimeoutMs);
    rMaster.setSelectedSensorPosition(0, Constants.VELOCITY_PID_INDEX, Constants.kTimeoutMs);
  }

  public double getLeftEncoderPos() { //in inches
    return lMaster.getSelectedSensorPosition(Constants.VELOCITY_PID_INDEX)/Constants.ENCODER_TICK_ROTATION*Constants.WHEEL_CIRCUMFERENCE;
  }

  public double getRightEncoderPos() { //in inches
    return rMaster.getSelectedSensorPosition(Constants.VELOCITY_PID_INDEX)/Constants.ENCODER_TICK_ROTATION*Constants.WHEEL_CIRCUMFERENCE;
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

  public enum Drivemode {
    percentOutput, velocity, percentOutputTurnControl;
  }
}
