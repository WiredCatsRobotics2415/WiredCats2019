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

	public static final int INTAKE_PCM_ID = 21;

	//Drivetrain
    public static final int LEFT_TALON_BACK = 11; //14
	public static final int LEFT_TALON_FRONT = 2; 
	public static final int RIGHT_TALON_BACK = 1; 
	public static final int RIGHT_TALON_FRONT = 3;
	//competition bot
	public static final boolean LEFT_TALON_BACK_DIRECTION = false; //true
	public static final boolean LEFT_TALON_FRONT_DIRECTION = true;
	public static final boolean RIGHT_TALON_BACK_DIRECTION = false;
	public static final boolean RIGHT_TALON_FRONT_DIRECTION = true; //false

	public static final boolean PIGEON_ON_CAN = false;
	public static final int PIGEON_ID = 3; //need to fix (-1 for null)
	public static final YawPitchRoll PIGEON_DIRECTION = YawPitchRoll.Yaw;
	public static final YawPitchRoll NAVX_DIRECTION = YawPitchRoll.Yaw;
	public static enum YawPitchRoll {
		Yaw, Pitch, Roll;
	}

	public static final int LEFT_DRIVETRAIN_ENCODER = 11;
	public static final int RIGHT_DRIVETRAIN_ENCODER = 1;
	
	//practice bot
	/*public static final boolean LEFT_TALON_BACK_DIRECTION = true;
	public static final boolean LEFT_TALON_FRONT_DIRECTION = true;
	public static final boolean RIGHT_TALON_BACK_DIRECTION = false;
	public static final boolean RIGHT_TALON_FRONT_DIRECTION = false;*/
	
	//Elevator
	public static final int ELEVATOR_ONE = 4;
	public static final int ELEVATOR_TWO = 13;
	public static final int ELEVATOR_THREE = 14; //11
	public static final int ELEVATOR_FOUR = 15;

	public static final boolean ELEVATOR_ONE_DIRECTION = false;
	public static final boolean ELEVATOR_TWO_DIRECTION = false;
	public static final boolean ELEVATOR_THREE_DIRECTION = true;
	public static final boolean ELEVATOR_FOUR_DIRECTION = true;

	public static final int ELEV_BOT = 1;

	public static final int ELEV_SWITCH_1 = 1;
	public static final int ELEV_SWITCH_2 = 3;

	public static final int EXTENDY_1 = 0;
	public static final int EXTENDY_2 = 2;

	public static final int STRETCHY_1 = 4;
	public static final int STRETCHY_2 = 6;

	//Intake
	public static final int INTAKE = 5;
	public static final int INTAKE_ROTATE = 0;
	public static final int INTAKE_HOLDER_1 = 4;
	public static final int INTAKE_HOLDER_2 = 5;

	//Climber
	public static final int CLIMB_PISTON_1 = 5;
	public static final int CLIMB_PISTON_2 = 7;
}
