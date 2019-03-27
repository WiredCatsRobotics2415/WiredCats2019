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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
    intake = new Intake();
    intakeRotator = new IntakeRotator();
    elevator = new Elevator();
    // endgame = new Endgame();
    hatchManip = new HatchManipulator();

    // limelight = new Limelight();
    // compressor.start();
    compressor.setClosedLoopControl(true);
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
    // SmartDashboard.putBoolean("EXTENDED?", hatchManip.isOut());
    // SmartDashboard.putBoolean("STRETCHED?", hatchManip.isStretched());
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
    // elevator.shiftUp();
    // endgame.flipIn();
    drivetrain.setBrakeMode(true);
    // hatchManip.retract();
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
    // elevator.setBrakeMode(true);
    // elevator.shiftUp();
    // endgame.flipIn();
    // endgame.stop();
    drivetrain.setBrakeMode(true);
    // hatchManip.stretch();
    // hatchManip.extend();
    
    // drivetrain.enableSafeties();
    // compressor.start();
    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // System.out.println(compressor.enabled());
    // System.out.println(compressor.getCompressorCurrent());

    double leftY, rightX;
    leftY = Math.abs(gamepad.getRawAxis(1)) > drivetrain.DEADBAND ? gamepad.getRawAxis(1) : 0;
    rightX = Math.abs(-gamepad.getRawAxis(2)) > drivetrain.DEADBAND ? -gamepad.getRawAxis(2) : 0;

    boolean isQuickTurn = Math.abs(leftY) < 0.1 && Math.abs(rightX) >= .1;

    if (gamepad.getRawButtonPressed(3)) {
      System.out.println("LEFT: " + leftY);
      drivetrain.printCurrent();
    }

    // drivetrain.drive(0.9*leftY, -0.9*rightX);
    drivetrain.babyDrive(-0.9*leftY, 0.9*rightX);

    if (gamepad.getRawButtonPressed(11)) {
      System.out.println("EXTEND");
      hatchManip.extendToggle();
    } else if (gamepad.getRawButtonPressed(12)) {
      System.out.println("STRETCH");
      hatchManip.stretchToggle();
    }

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

    // if (gamepad.getRawButton(7)) {
    //   intakeRotator.setMotor(-1);
    // } else if (gamepad.getRawButton(8)) {
    //   intakeRotator.setMotor(1);
    // } else {
      
    // }


    double elevatorspeed = 0;
    // System.out.println(leftY);
    // if (Math.abs(elevatorspeed) < 0.15) elevatorspeed = 0;

    if (gamepad.getPOV() == 90) {
      elevatorspeed = -0.7;
      // System.out.println("HELLO");
    } else if (gamepad.getPOV() == 270) {
      elevatorspeed = 0.7;
    }

    elevator.setElevMotors(elevatorspeed);

    // if (gamepad.getRawButton(2)) { //x
    //   elevator.shiftDown();
    // } else if (gamepad.getRawButton(1)) { //square
    //   elevator.shiftUp();
    // }

    // if (gamepad.getRawButtonPressed(14)) { //touchpad
    //   endgame.flipOut();
    // } else if (gamepad.getRawButtonPressed(13)) { //PS button home button
    //   endgame.flipIn();
    // }

    // if (gamepad.getRawButtonPressed(3)) { //circle
    //   intakeRotator.sendOut();
    // } else if (gamepad.getRawButtonPressed(4)) { //triangle
    //   intakeRotator.holdIn();
    // }

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    double leftY = gamepad.getRawAxis(1);
    // drivetrain.setMotors(leftY, leftY);
    // endgame.testMotor(leftY);
    System.out.println(leftY);
    drivetrain.testMotor(leftY);
  }

  @Override
  public void disabledPeriodic() {
    // intakeRotator.setBrakeMode(false);
    // elevator.setBrakeMode(false);
    // System.out.println(drivetrain.getYaw());
    // System.out.println(gamepad.getPOV());
    // System.out.println(gamepad);
    // System.out.println(elevator.getTop());
    // System.out.println(gamepad.get)
  }
}
