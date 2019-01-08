/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ArcadeDrive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX lFront, rFront, lBack, rBack;
  public AHRS ahrs;

  private final double WHEEL_CIRCUMFERENCE = Math.PI * 8; //inches


  public ArcadeDrive() {
    lFront = new WPI_TalonSRX(RobotMap.LEFT_TALON_FRONT);
    rFront = new WPI_TalonSRX(RobotMap.RIGHT_TALON_FRONT);
    lBack = new WPI_TalonSRX(RobotMap.LEFT_TALON_BACK);
    rBack = new WPI_TalonSRX(RobotMap.RIGHT_TALON_BACK);

  }
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
