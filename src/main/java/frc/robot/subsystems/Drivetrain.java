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

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.RobotMap;
import frc.robot.cheesy.DriveSignal;

public class Drivetrain extends Subsystem {
  //motors and sensors
  private WPI_TalonSRX lFront, rFront, lBack, rBack;
  public AHRS ahrs;

  //class varibles
  private WPI_TalonSRX lMaster, rMaster;
  private ControlMode controlMode;

  //constants
  private final double WHEEL_CIRCUMFERENCE = Math.PI * 8; //inches
  
  public final double DEADBAND = 0.05;
	public final float INTERPOLATION_FACTOR = 0.75f;   //Nathan's Settings
	public final float STRAIGHT_LIMITER = 0.95f;
	public final float TURN_BOOSTER = 1.3f;
  
  public Drivetrain() {
    lFront = new WPI_TalonSRX(RobotMap.LEFT_TALON_FRONT);
    rFront = new WPI_TalonSRX(RobotMap.RIGHT_TALON_FRONT);
    lBack = new WPI_TalonSRX(RobotMap.LEFT_TALON_BACK);
    rBack = new WPI_TalonSRX(RobotMap.RIGHT_TALON_BACK);

    lFront.setInverted(RobotMap.LEFT_TALON_FRONT_DIRECTION);
    lBack.setInverted(RobotMap.LEFT_TALON_BACK_DIRECTION);
    rFront.setInverted(RobotMap.RIGHT_TALON_FRONT_DIRECTION);
    rBack.setInverted(RobotMap.RIGHT_TALON_BACK_DIRECTION);

    lFront.follow(lBack);
    rFront.follow(rBack);
    lMaster = lBack;
    rMaster = rBack;

    setBrakeMode(true);

    lMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    setControlMode(ControlMode.PercentOutput);
    ahrs = null;
  }

  public void setMotors(double left, double right) {
    lMaster.set(controlMode, left);
    rMaster.set(controlMode, right);
  }

  public void drive(double throttle, double turn) {
    boolean isQuickTurn;
    if (Math.abs(throttle) < Math.abs(DEADBAND)) throttle = 0;
    if (Math.abs(turn) < Math.abs(DEADBAND)) turn = 0;
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

  public void setControlMode(ControlMode controlMode) {
    this.controlMode = controlMode;
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

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
