package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class OI {
    private XboxController gamepad;

    public OI(XboxController gamepad) {
        this.gamepad = gamepad;
    }
    //Drivetrain
    public double getDriveThrottle() {
        return gamepad.getRawAxis(1);
    }
    public double getDriveTurn() {
        return gamepad.getRawAxis(2);
    }

    //Elevator
    public boolean getElevatorUp() {
        return gamepad.getPOV() == 270;
    }
    public boolean getElevatorDown() {
        return gamepad.getPOV() == 90;
    }

    //Hatch Maniupulator
    public boolean getHatchExtendToggle() {
        return gamepad.getRawButtonPressed(11);   
    }
    public boolean getHatchStretchToggle() {
        return gamepad.getRawButtonPressed(12);
    }

    //Intake
    public boolean getIntaking() {
        return false; //TODO
    }
    public boolean getOutaking() {
        return false; //TODO
    }

    //Intake Rotator
    public boolean getIntakeRotatorUp() {
        return gamepad.getRawButton(7);
    }
    public boolean getIntakeRotatorDown() {
        return gamepad.getRawButton(8);
    }

    //Endgame
    public boolean getEndgameFlipOut() {
        return gamepad.getRawButton(14);
    }
    public boolean getEndgameFlipIn() {
        return gamepad.getRawButton(13);
    }
}