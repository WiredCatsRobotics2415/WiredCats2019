package frc.robot;

import frc.robot.util.pid.PIDValue;

public class Constants {
    //physical constants
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * 8; //inches
    public static final double DRIVETRAIN_MAX_SPEED = 3628.0;

    //sensor contants
    public static int ENCODER_TICK_ROTATION = 4096;

    //drivetrain
    public static final double DEADBAND = 0.05;
	public static final float INTERPOLATION_FACTOR = 0.75f;   //Nathan's Settings
	public static final float STRAIGHT_LIMITER = 0.95f;
    public static final float TURN_BOOSTER = 1.3f;
    
    //PID Drivetrain
    public static final int kTimeoutMs = 30;
    public static int VELOCITY_PID_INDEX = 0;
    public static final PIDValue VELOCITY_PID = new PIDValue(1023/DRIVETRAIN_MAX_SPEED, 0.69, 0.0, 0.1);
}