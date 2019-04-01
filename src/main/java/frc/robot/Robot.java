/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;
import java.util.HashMap;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.auto.FrontCargoShip;
import frc.robot.auto.PassLine;
import frc.robot.cheesy.CheesyDriveHelper;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeRotator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Endgame;
import frc.robot.subsystems.HatchManipulator;

import frc.util.Limelight;
import frc.util.Limelight.LightMode;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
  private static final AutoChoice defaultAuto = AutoChoice.cameraControl;
  private static final StartLocation defaultStart = StartLocation.middle;
  private AutoChoice autoSelelected;
  private StartLocation startLocationSelected;
  private ShuffleboardTab autoSelector;
  private SendableChooser<AutoChoice> autoChooser;
  private SendableChooser<StartLocation> startLocation;

  public static OI oi;
  public static Compressor compressor;
  public static CheesyDriveHelper cheesyDriveHelper;

  public static Drivetrain drivetrain;
  public static Intake intake;
  public static IntakeRotator intakeRotator;
  public static Elevator elevator;
  public static Endgame endgame;
  public static HatchManipulator hatchManip;

  public static Map<Integer,WPI_TalonSRX> robotTalons = new HashMap<Integer,WPI_TalonSRX>();

  public static Relay ringlight;

  public static Limelight limelight;

  private boolean limelightOn = false, endgameOn = false;

  public enum AutoChoice {
    leftFrontCargo("Left Front Cargo"), rightFrontCargo("Right Front Cargo"), passLine("Pass Line"), cameraControl("Camera Control");
    
    private final String name;
    private AutoChoice(String name) {
      this.name = name;
    }
    @Override
    public String toString() {
      return name;
    }
  }

  public enum StartLocation {
    left("Left"), left_Level_2("Left Level 2"), middle("Middle"), right("Right"), right_Level_2("Right Level 2");
    
    private final String name;
    private StartLocation(String name) {
      this.name = name;
    }
    @Override
    public String toString() {
      return name;
    }
  }

  @Override
  public void robotInit() {
    autoSelector = Shuffleboard.getTab("Auto Selector");
    autoChooser = new SendableChooser<AutoChoice>();
    startLocation = new SendableChooser<StartLocation>();
    autoChooser.setDefaultOption(defaultAuto.toString(), defaultAuto);
    for(int i = 0; i < AutoChoice.values().length; i++) {
      if(AutoChoice.values()[i] == defaultAuto) continue;
      autoChooser.addOption(AutoChoice.values()[i].toString(), AutoChoice.values()[i]);
    }
    startLocation.setDefaultOption(defaultStart.toString(), defaultStart);
    for(int i = 0; i < StartLocation.values().length; i++) {
      if(StartLocation.values()[i] == defaultStart) continue;
      startLocation.addOption(StartLocation.values()[i].toString(), StartLocation.values()[i]);
    }
    // autoSelector.add(autoChooser);
    // autoSelector.add(startLocation);
    autoSelelected = defaultAuto;
    startLocationSelected = defaultStart;

    oi = new OI(new XboxController(0));
    // operator = new XboxController(1);
    compressor = new Compressor(RobotMap.PCM_ID);

    cheesyDriveHelper = new CheesyDriveHelper();

    drivetrain = new Drivetrain();
    intake = new Intake();
    intakeRotator = new IntakeRotator();
    elevator = new Elevator();
    endgame = new Endgame();
    hatchManip = new HatchManipulator();

    limelight = new Limelight();
    // compressor.start();
    compressor.setClosedLoopControl(true);
    compressor.stop();

    limelight.setLedMode(LightMode.eOff);
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
    
    // autoSelelected = autoChooser.getSelected();
    // startLocationSelected = startLocation.getSelected();
    // System.out.println("Auto selected: " + autoSelelected.toString());
    // System.out.println("Start Position" + startLocationSelected.toString());
    // Command autoCommand = null;
    // switch(autoSelelected) {
    //   case leftFrontCargo:
    //     autoCommand = new FrontCargoShip(startLocationSelected, true); 
    //     break;
    //   case rightFrontCargo:
    //     autoCommand = new FrontCargoShip(startLocationSelected, false);
    //     break;
    //   case passLine:
    //     autoCommand = new PassLine(startLocationSelected);
    //   default:
    //     autoCommand = null;
    //     break;
    // }
    // if(autoCommand != null) {
    //   autoCommand.start();
    // }
    // System.out.println("Auto selected: " + m_autoSelected);
    // intakeRotator.setBrakeMode(true);

    elevator.setBrakeMode(true);
    // endgame.flipIn();
    drivetrain.setBrakeMode(true);
    endgameOn = false;
    limelight.setLedMode(LightMode.eOff);
    elevator.shiftUp();
    // hatchManip.retract();
    // hatchManip.shrink();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
      teleopPeriodic();
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
    // hatchManip.retract();
    // hatchManip.shrink();
    endgame.flipIn();
    // endgame.stop();
    limelight.setLedMode(LightMode.eOff);
    drivetrain.setBrakeMode(true);
    endgameOn = false;
    
    // drivetrain.enableSafeties();
    // compressor.start();
    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // if (oi.getButton(4)) {
    //   controllerLimelightDrive();
    // } else {
    //   controllerDrivetrain();
    // }    
    controllerDrivetrain();
    // controllerHatchMan();
    // controllerIntake();
    // controllerIntakeRotator();
    controllerEndgame();
    controllerElevator();

    if (oi.getButton(1)) {
      drivetrain.printCurrent();
      System.out.println();
    }
    
  }

  private void controllerDrivetrain() {
    double throttle, turn;
    throttle = oi.getDriveThrottle();
    turn = oi.getDriveTurn();

    limelight.setLedMode(LightMode.eOff);

    drivetrain.drive(throttle, turn);

    //drivetrain.drive(cheesyDriveHelpl[er.cheesyDrive(leftY, rightX, isQuickTurn, false));  
    
    // if (Math.abs(turn) > 0.08) {
    //   drivetrain.setTargetYaw(-1000);
    //   drivetrain.drive(throttle, turn);
    // } else {
    //   if (drivetrain.getTargetYaw() == -1000) {
    //     drivetrain.setTargetYaw(drivetrain.getYaw());
    //   }
    //   drivetrain.straightDrive(throttle);
    // }
  }

  private void controllerLimelightDrive() {
    limelight.setLedMode(LightMode.eOn);
    double throttle, turn;
    throttle = Math.abs(oi.getDriveThrottle()) > drivetrain.DEADBAND ? oi.getDriveThrottle() : 0;
    turn = Math.abs(oi.getDriveTurn()) > drivetrain.DEADBAND ? oi.getDriveTurn() : 0;

    double left, right;

    left = throttle - turn;
    right = throttle + turn;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    double x = tx.getDouble(0.00);
    double y = ty.getDouble(0.00);
    double area = ta.getDouble(0.00);

    if (limelight.isTarget()) {
      // System.out.println("HEADING ERROR: " + x);

      double heading_error = x;
      double distance_error = area - limelight.TARGET_AREA;
      // System.out.println("HEADING ERROR: " + heading_error);
      double steering_adjust = 0.0;
      double distance_adjust = 0.0;

      if (x > 1.0) {
        steering_adjust = limelight.kP_TURN * heading_error; //+ limelight.min_command;
      } else if (x < 1.0) {
        steering_adjust = limelight.kP_TURN * heading_error; // - limelight.min_command;
      }

      // distance_adjust = distance_error * 0.05;

      left += steering_adjust + distance_adjust;
      right -= steering_adjust - distance_adjust;

      // System.out.println("STEERING ADJUST: " + steering_adjust);

      // System.out.println("DISTANCE: " + distance_error);

    }

    drivetrain.setMotorsInd(left, right);
  }

  private void controllerIntake() {
    if (oi.getIntaking()) {
      intake.intake();
    } else if (oi.getOutaking()) {
      intake.outtake();
    } else {
      intake.still();
    }
  }

  private void controllerIntakeRotator() {
    if (oi.getIntakeRotatorUp()) {
      intakeRotator.rotateUp();
    } else if (oi.getIntakeRotatorDown()) {
      intakeRotator.rotateDown();
    } else {
      intakeRotator.stop();
    }

    if (oi.getButtonPressed(3)) {
      intakeRotator.toggleHold();
    }
  }

  private void controllerElevator() {
    if (oi.getElevatorDown()) {
      if(elevator.getEndgameShift()) {
        elevator.endgameLower();
      } else {
        elevator.lowerDown();
      }
    } else if (oi.getElevatorUp()) {
      elevator.liftUp();
    } else {
      elevator.stop();
    }

    if (oi.getButtonPressed(1)) { //square
      elevator.shiftUp();
      endgameOn = false;
    } else if (oi.getButtonPressed(2)) { //x
      elevator.shiftDown();
      endgameOn = true;
    }
  }

  private void controllerEndgame() {
    if (oi.getEndgameFlipOut()) { //touchpad
      endgame.flipOut();
      elevator.shiftDown();
      endgameOn = true;
    } else if (oi.getEndgameFlipIn()) { //PS button home button
      endgame.flipIn();
      elevator.shiftUp();      
      endgameOn = false;
    }
  }

  private void controllerHatchMan() {
    if (oi.getHatchExtendToggle() && !hatchManip.isStretched()) {//left joystick 
      hatchManip.extendToggle();
    }
    if (oi.getHatchStretchToggle() && hatchManip.isOut()) { // right joystick 
      hatchManip.stretchToggle();
    }
  }

  public static WPI_TalonSRX getTalon(int deviceNumber) {
    if(robotTalons.containsKey(deviceNumber)) {
      return robotTalons.get(deviceNumber);
    }
    WPI_TalonSRX newTalon = new WPI_TalonSRX(deviceNumber);
    robotTalons.put(deviceNumber, newTalon);
    return newTalon;
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    double leftY = -oi.getDriveThrottle();
    // drivetrain.setMotors(leftY, leftY);
    // endgame.testMotor(leftY);
    System.out.println(leftY);
    elevator.testElev(leftY);
  }

  @Override
  public void disabledPeriodic() {
    intakeRotator.holdIn();
    limelight.setLedMode(LightMode.eOff);
    // System.out.println("Yaw: " + drivetrain.getYaw());
    // intakeRotator.setBrakeMode(false);
    // elevator.setBrakeMode(false);
    // System.out.println(drivetrain.getYaw());
    // System.out.println(gamepad.getPOV());
    // System.out.println(gamepad);
    // System.out.println(elevator.getTop());
    // System.out.println(gamepad.get)
  }
}
