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
import frc.robot.subsystems.Endgame;
import frc.robot.subsystems.HatchManipulator;
import frc.util.Limelight;
import edu.wpi.first.cameraserver.CameraServer;
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
  public static Endgame endgame;
  public static HatchManipulator hatchManip;

  public static Relay ringlight;

  public static Limelight limelight;

  private boolean limelightOn;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    // CameraServer.getInstance().startAutomaticCapture();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    gamepad = new XboxController(0);
    // operator = new XboxController(1);
    compressor = new Compressor(RobotMap.PCM_ID);

    cheesyDriveHelper = new CheesyDriveHelper();

    drivetrain = new Drivetrain();
    // intake = new Intake();
    // intakeRotator = new IntakeRotator();
    elevator = new Elevator();
    // endgame = new Endgame();
    hatchManip = new HatchManipulator();

    // limelight = new Limelight();
    // compressor.stop();

    limelightOn = false;

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
    // System.out.println("Auto selected: " + m_autoSelected);
    // intakeRotator.setBrakeMode(true);

    elevator.setBrakeMode(true);
    elevator.shiftUp();
    // endgame.flipIn();
    // endgame.stop();
    drivetrain.setBrakeMode(true);
    hatchManip.stretch();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    double leftY, rightX;
    leftY = Math.abs(gamepad.getRawAxis(1)) > drivetrain.DEADBAND ? gamepad.getRawAxis(1) : 0;
    rightX = Math.abs(gamepad.getRawAxis(2)) > drivetrain.DEADBAND ? gamepad.getRawAxis(2) : 0;

    boolean isQuickTurn = Math.abs(leftY) < 0.1 && Math.abs(rightX) >= .1;

    if (gamepad.getRawButtonPressed(3)) {
      System.out.println("LEFT: " + leftY);
      drivetrain.printCurrent();
    }
    // drivetrain.drive(0.9*leftY, -0.9*rightX);
    drivetrain.babyDrive(0.9*leftY, -0.9*rightX);
    // if (gamepad.getBumperPressed(Hand.kRight)) {
      // drivetrain.enableSafeties();
      // drivetrain.disableSafeties();
    // }
    // System.out.println(leftY);
    // drivetrain.drive(cheesyDriveHelper.cheesyDrive(leftY, rightX, isQuickTurn, false));
    
    // if (gamepad.getBumper(Hand.kLeft) ) {
    //     intake.intake();
    // } else if (gamepad.getBumper(Hand.kRight)) {
    //     intake.outtake();
    // } else {
    //     intake.still();
    // }

    // double leftTrigger, rightTrigger;

    // leftTrigger = gamepad.getRawAxis(3);
    // if (leftTrigger < 0) leftTrigger = 0;

    // rightTrigger = gamepad.getRawAxis(4);
    // if (rightTrigger < 0) rightTrigger = 0;

    // if (leftTrigger > 0) {
    //   intakeRotator.setMotor(-1*leftTrigger);
    // } else if (rightTrigger > 0) {
    //   intakeRotator.setMotor(rightTrigger);
    // } else {
    //   double rotate;
    //   rotate = operator.getRawAxis(5);
    //   if (Math.abs(rotate) < 0.15 ) rotate = 0;
    //   intakeRotator.setMotor(rotate);
    // }


    double elevatorspeed = 0;
    if (Math.abs(elevatorspeed) < 0.15) elevatorspeed = 0;

    if (gamepad.getPOV() == 0) {
      elevatorspeed = -0.72;
      // System.out.println("HELLO");
    } else if (gamepad.getPOV() == 180) {
      elevatorspeed = 0.5;
    } else if (gamepad.getPOV() == -1) {
      // elevatorspeed = -operator.getRawAxis(1);
      // if (Math.abs(elevatorspeed) < 0.15) elevatorspeed = 0;
    }

    // if (elevator.getTop() && elevatorspeed < 0) {
    //   // System.out.println("Hello");
    //   elevatorspeed = 0;
    // }

    // System.out.println(elevatorspeed);[]\

    elevator.setElevMotors(elevatorspeed);

    // if (gamepad.getRawButton(2)) {
    //   elevator.shiftDown();
    // } else if (gamepad.getRawButton(1)) {
    //   elevator.shiftUp();
    // }

    // if (gamepad.getRawButton(14)) {
    //   endgame.flipOut();
    //   endgame.spin();
    // } else if (gamepad.getRawButton(13)) { //PS button home button
    //   endgame.flipIn();
    //   endgame.stop();
    // }


    if (gamepad.getBumperPressed(Hand.kRight)) { //left joystick
      hatchManip.extendToggle();
    }
    
    if (gamepad.getBumperPressed(Hand.kLeft)) { // right joystick
      hatchManip.stretchToggle();
    }

  
}

  /**
   * This function is called periodically during test mode.
   */


  /**
   * This function is called at the beginning of operator control.
   */
  @Override
  public void teleopInit() {
    // intakeRotator.setBrakeMode(true);
    elevator.setBrakeMode(true);
    elevator.shiftUp();
    // endgame.flipIn();
    // endgame.stop();
    drivetrain.setBrakeMode(true);
    hatchManip.stretch();
    
    // drivetrain.enableSafeties();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    double leftY, rightX;
    leftY = Math.abs(gamepad.getRawAxis(1)) > drivetrain.DEADBAND ? gamepad.getRawAxis(1) : 0;
    rightX = Math.abs(gamepad.getRawAxis(2)) > drivetrain.DEADBAND ? gamepad.getRawAxis(2) : 0;

    boolean isQuickTurn = Math.abs(leftY) < 0.1 && Math.abs(rightX) >= .1;

    if (gamepad.getRawButtonPressed(3)) {
      System.out.println("LEFT: " + leftY);
      drivetrain.printCurrent();
    }
    // drivetrain.drive(0.9*leftY, -0.9*rightX);
    drivetrain.babyDrive(0.9*leftY, -0.9*rightX);
    // if (gamepad.getBumperPressed(Hand.kRight)) {
      // drivetrain.enableSafeties();
      // drivetrain.disableSafeties();
    // }
    // System.out.println(leftY);
    // drivetrain.drive(cheesyDriveHelper.cheesyDrive(leftY, rightX, isQuickTurn, false));
    
    // if (gamepad.getBumper(Hand.kLeft) ) {
    //     intake.intake();
    // } else if (gamepad.getBumper(Hand.kRight)) {
    //     intake.outtake();
    // } else {
    //     intake.still();
    // }

    // double leftTrigger, rightTrigger;

    // leftTrigger = gamepad.getRawAxis(3);
    // if (leftTrigger < 0) leftTrigger = 0;

    // rightTrigger = gamepad.getRawAxis(4);
    // if (rightTrigger < 0) rightTrigger = 0;

    // if (leftTrigger > 0) {
    //   intakeRotator.setMotor(-1*leftTrigger);
    // } else if (rightTrigger > 0) {
    //   intakeRotator.setMotor(rightTrigger);
    // } else {
    //   double rotate;
    //   rotate = operator.getRawAxis(5);
    //   if (Math.abs(rotate) < 0.15 ) rotate = 0;
    //   intakeRotator.setMotor(rotate);
    // }


    double elevatorspeed = 0;
    if (Math.abs(elevatorspeed) < 0.15) elevatorspeed = 0;

    if (gamepad.getPOV() == 0) {
      elevatorspeed = -0.72;
      // System.out.println("HELLO");
    } else if (gamepad.getPOV() == 180) {
      elevatorspeed = 0.5;
    } else if (gamepad.getPOV() == -1) {
      // elevatorspeed = -operator.getRawAxis(1);
      // if (Math.abs(elevatorspeed) < 0.15) elevatorspeed = 0;
    }

    // if (elevator.getTop() && elevatorspeed < 0) {
    //   // System.out.println("Hello");
    //   elevatorspeed = 0;
    // }

    // System.out.println(elevatorspeed);[]\

    elevator.setElevMotors(elevatorspeed);

    // if (gamepad.getRawButton(2)) {
    //   elevator.shiftDown();
    // } else if (gamepad.getRawButton(1)) {
    //   elevator.shiftUp();
    // }

    // if (gamepad.getRawButton(14)) {
    //   endgame.flipOut();
    //   endgame.spin();
    // } else if (gamepad.getRawButton(13)) { //PS button home button
    //   endgame.flipIn();
    //   endgame.stop();
    // }


    if (gamepad.getBumperPressed(Hand.kRight)) { //left joystick
      hatchManip.extendToggle();
    }
    
    if (gamepad.getBumperPressed(Hand.kLeft)) { // right joystick
      hatchManip.stretchToggle();
    }

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    // double leftY = gamepad.getRawAxis(1);
    // drivetrain.setMotors(leftY, leftY);
    // endgame.testMotor(leftY);
    // System.out.println(elevator.getTop());
  }

  @Override
  public void disabledPeriodic() {
    // intakeRotator.setBrakeMode(false);
    // elevator.setBrakeMode(false);
    // System.out.println(drivetrain.getYaw());
    // System.out.println(gamepad.getPOV());
    // System.out.println(elevator.getTop());
  }
}
