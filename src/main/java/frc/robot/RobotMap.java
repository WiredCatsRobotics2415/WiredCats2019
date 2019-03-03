/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class RobotMap {

    public static final int PCM_ID = 20;
	//Drivetrain
	public static final int LEFT_TALON_BACK = 14; 
	public static final int LEFT_TALON_FRONT = 15; 
	public static final int RIGHT_TALON_BACK = 1; 
	public static final int RIGHT_TALON_FRONT = 3;
	//competition bot
	public static final boolean LEFT_TALON_BACK_DIRECTION = true;
	public static final boolean LEFT_TALON_FRONT_DIRECTION = true;
	public static final boolean RIGHT_TALON_BACK_DIRECTION = false;
	public static final boolean RIGHT_TALON_FRONT_DIRECTION = false;
	//practice bot
	/*public static final boolean LEFT_TALON_BACK_DIRECTION = true;
	public static final boolean LEFT_TALON_FRONT_DIRECTION = true;
	public static final boolean RIGHT_TALON_BACK_DIRECTION = false;
	public static final boolean RIGHT_TALON_FRONT_DIRECTION = false;*/

	public static final int ELEVATOR_ONE = 12;
	public static final int ELEVATOR_TWO = 13;
	public static final int ELEVATOR_THREE = 6; //no
	public static final int ELEVATOR_FOUR = 6; //no

	public static final int ELEV_TOP = 1;
	public static final int ELEV_BOT = 0;

	public static final int INTAKE = 2;
	public static final int INTAKE_ROTATE = 0;

}
