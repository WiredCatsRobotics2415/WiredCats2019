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
        return Math.abs(gamepad.getRawAxis(1)) > 0.05 ? gamepad.getRawAxis(1) : 0;
    }
    public double getDriveTurn() {
        return Math.abs(-gamepad.getRawAxis(2)) > 0.05 ? -gamepad.getRawAxis(2) : 0;
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
        return gamepad.getBumper(Hand.kLeft);
    }
    public boolean getOutaking() {
        return gamepad.getBumper(Hand.kRight);
    }

    //Intake Rotator
    public boolean getIntakeRotatorUp() {
        return gamepad.getRawButton(8);
    }
    public boolean getIntakeRotatorDown() {
        return gamepad.getRawButton(7);
    }

    //Endgame
    public boolean getEndgameFlipOut() {
        return gamepad.getRawButton(14);
    }
    public boolean getEndgameFlipIn() {
        return gamepad.getRawButton(13);
    }

    public boolean getButtonPressed(int button) {
        return gamepad.getRawButtonPressed(button);
    }

    public boolean getButton(int button) {
        return gamepad.getRawButton(button);
    }
}