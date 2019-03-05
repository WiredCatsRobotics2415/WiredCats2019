/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.cheesy.CheesyDriveHelper;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeRotator;
import frc.robot.subsystems.Elevator;
import frc.util.Limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static XboxController gamepad;
  public static XboxController operator;
  public static Compressor compressor;
  public static CheesyDriveHelper cheesyDriveHelper;

  public static Drivetrain drivetrain;
  public static Intake intake;
  public static IntakeRotator intakeRotator;
  public static Elevator elevator;

  public static Relay ringlight;

  public static Limelight limelight;

  private double lastPrint;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    gamepad = new XboxController(0);
    operator = new XboxController(1);
    compressor = new Compressor(RobotMap.PCM_ID);

    cheesyDriveHelper = new CheesyDriveHelper();

    drivetrain = new Drivetrain();
    intake = new Intake();
    intakeRotator = new IntakeRotator();
    elevator = new Elevator();

    // limelight = new Limelight();

    // compressor.stop();
    // ringlight = new Relay(0);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    double leftY, rightX;
    leftY = gamepad.getRawAxis(1);
    rightX = -gamepad.getRawAxis(4);

    boolean isQuickTurn = Math.abs(leftY) < 0.1 && Math.abs(rightX) >= .1;

    //drivetrain.drive(cheesyDriveHelper.cheesyDrive(leftY, rightX, isQuickTurn, false));  
    drivetrain.drive(leftY, rightX);

    double rotate = operator.getRawAxis(1);
    if (Math.abs(rotate) < Constants.DEADBAND) rotate = 0;
    intakeRotator.setMotor(rotate);

    if (gamepad.getBumper(Hand.kLeft)) {
        intake.intake();
    } else if (gamepad.getBumper(Hand.kRight)) {
        intake.outtake();
    } else if (operator.getBumper(Hand.kLeft)) {
        intake.intake();
    } else if (operator.getBumper(Hand.kRight)) {
        intake.outtake();
    } else {
        intake.still();
    }

    double elevatorspeed = operator.getRawAxis(1);
    if (Math.abs(elevatorspeed) < Constants.DEADBAND) elevatorspeed = 0;
    elevator.setElevMotors(7/drivetrain.getBusVoltage()*elevatorspeed);

  }

  /**
   * This function is called at the beginning of operator control.
   */
  @Override
  public void teleopInit() {
    intakeRotator.setBrakeMode(true);
    elevator.setBrakeMode(true);
    elevator.shiftUp();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    double leftY, rightX;
    leftY = gamepad.getRawAxis(1);
    rightX = -gamepad.getRawAxis(4);

    boolean isQuickTurn = Math.abs(leftY) < 0.1 && Math.abs(rightX) >= .1;

    //drivetrain.drive(cheesyDriveHelper.cheesyDrive(leftY, rightX, isQuickTurn, false));  
    drivetrain.drive(leftY, rightX);
/*
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.00);
    double y = ty.getDouble(0.00);
    double area = ta.getDouble(0.00);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    if (limelight.isTarget()) {
      double heading_error = x;
      double distance_error = area - limelight.TARGET_AREA;
      // System.out.println("HEADING ERROR: " + heading_error);
      double steering_adjust = 0.0;
      double distance_adjust = 0.0;

      if (x > 1.0) {
        steering_adjust = limelight.kP * heading_error; //+ limelight.min_command;
      } else if (x < 1.0) {
        steering_adjust = limelight.kP * heading_error; // - limelight.min_command;
      }

      distance_adjust = distance_error * 0.05;

      left += steering_adjust + distance_adjust;
      right -= steering_adjust - distance_adjust;

      // System.out.println("STEERING ADJUST: " + steering_adjust);

      // System.out.println("DISTANCE: " + distance_error);

    }
    */

    double rotate;

    if (gamepad.getBumper(Hand.kLeft)) {
        intake.intake();
    } else if (gamepad.getBumper(Hand.kRight)) {
        intake.outtake();
    } else if (operator.getBumper(Hand.kLeft)) {
        intake.intake();
    } else if (operator.getBumper(Hand.kRight)) {
        intake.outtake();
    } else {
        intake.still();
    }

    // double leftTrigger, rightTrigger;

    // leftTrigger = -gamepad.getTriggerAxis(Hand.kLeft);
    // if (Math.abs(leftTrigger) < arcadeDrive.DEADBAND) leftTrigger = 0;

    // rightTrigger = gamepad.getTriggerAxis(Hand.kRight);
    // if (Math.abs(rightTrigger) < arcadeDrive.DEADBAND) rightTrigger = 0;

    // if (Math.abs(leftTrigger) > arcadeDrive.DEADBAND) {
    //   System.out.println(leftTrigger);
    //   intakeRotator.setMotor(leftTrigger);
    // } else if (Math.abs(rightTrigger) > arcadeDrive.DEADBAND) {
    //   System.out.println(rightTrigger);
    //   intakeRotator.setMotor(rightTrigger);
    // } else {
    rotate = operator.getRawAxis(5);
    if (Math.abs(rotate) < 0.15 ) rotate = 0;
    intakeRotator.setMotor(rotate);
      // System.out.println(rotate);
    // }

    double elevatorspeed = operator.getRawAxis(1);
    if (Math.abs(elevatorspeed) < Constants.DEADBAND) elevatorspeed = 0;
    elevator.setElevMotors(7/drivetrain.getBusVoltage()*elevatorspeed);

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    double leftY = gamepad.getRawAxis(1);
    drivetrain.setMotors(leftY, leftY);
  }

  @Override
  public void disabledPeriodic() {
    intakeRotator.setBrakeMode(false);
    elevator.setBrakeMode(false);

    // System.out.println(arcadeDrive.getYaw());

    // System.out.println(elevator.getTop());
  }
}
