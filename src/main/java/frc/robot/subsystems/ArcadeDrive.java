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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.cheesy.DriveSignal;

/**
 * Add your docs here.
 */
public class ArcadeDrive extends Subsystem implements PIDOutput {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX lFront, rFront, lBack, rBack;
  public AHRS ahrs;

  private PIDController turnController;
  private double rotateToAngleRate;
  private double kP = .025;
	private double kI = 0.0025;
	private double kD = .0;
	private double kF = 0;
  private double kTolerance = 2.0;


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

    try {
      ahrs = new AHRS(Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    turnController = new PIDController(kP, kI, kD, kF, ahrs, this);

    //practice bot
    // lFront.setInverted(true);
    // lBack.setInverted(true);
    // rFront.setInverted(false);
    // rBack.setInverted(false);

    //competition bot
    lFront.setInverted(true);
    lBack.setInverted(true);
    rFront.setInverted(false);
    rBack.setInverted(false);

    lFront.follow(lBack);
    rFront.follow(rBack);

    lBack.set(ControlMode.PercentOutput, 0);
    rBack.set(ControlMode.PercentOutput, 0);

    lFront.set(ControlMode.PercentOutput, 0);
    rFront.set(ControlMode.PercentOutput, 0);

    lBack.setNeutralMode(NeutralMode.Coast);
    rBack.setNeutralMode(NeutralMode.Coast);

    lBack.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rBack.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

  }

  public void testMotor(double speed) {
    rFront.set(speed);
  }

  public void setMotors(double left, double right) {
    lBack.set(left);
    rBack.set(right);
    // lFront.set(left);
    // rFront.set(right);
  }

  public void setMotorsStraight(double left, double right, double yaw) {
    turnController.setInputRange(-180.0f,  180.0f);
    turnController.setOutputRange(-1.0, 1.0);
    turnController.setAbsoluteTolerance(kTolerance);
    turnController.setContinuous(true);
    turnController.setSetpoint(yaw);
    turnController.enable();
    lBack.set(left - rotateToAngleRate);
    rBack.set(right + rotateToAngleRate);
  }

  public double getBusVoltage() {
    return lBack.getBusVoltage();
  }

  public double[] getVelocity() {
    double[] velocities = new double[2];
    velocities[0] = lBack.getSelectedSensorVelocity();
    velocities[1] = rBack.getSelectedSensorVelocity();
    return velocities;

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
    if (brake) {
      lBack.setNeutralMode(NeutralMode.Brake);
      rBack.setNeutralMode(NeutralMode.Brake);
    } else {
      lBack.setNeutralMode(NeutralMode.Coast);
      rBack.setNeutralMode(NeutralMode.Coast);
    }

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

  @Override
  public void pidWrite(double output) {
		rotateToAngleRate = output;
	}

}
