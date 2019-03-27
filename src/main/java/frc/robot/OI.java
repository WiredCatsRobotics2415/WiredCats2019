package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class OI {
    private static final long bufferTimeMs = 50;
    private XboxController gamepad;
    private long hatchExtendBufferStartTimeMs;
    private long hatchStretchBufferStartTimeMs;

    public OI(XboxController gamepad) {
        this.gamepad = gamepad;
        this.hatchExtendBufferStartTimeMs = 0;
        this.hatchStretchBufferStartTimeMs = 0;
    }
    //Drivetrain
    public double getDriveThrottle() {
        return gamepad.getRawAxis(1);
    }
    public double getDriveTurn() {
        return gamepad.getRawAxis(0);
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
        if(gamepad.getBumperPressed(Hand.kRight)) {
            if(System.currentTimeMillis() > hatchExtendBufferStartTimeMs+bufferTimeMs) {
                hatchExtendBufferStartTimeMs = System.currentTimeMillis();
                return true;
            }
            return false;
        }
        return false;
    }
    public boolean getHatchStretchToggle() {
        if(gamepad.getBumperPressed(Hand.kLeft)) {
            if(System.currentTimeMillis() > hatchStretchBufferStartTimeMs+bufferTimeMs) {
                hatchStretchBufferStartTimeMs = System.currentTimeMillis();
                return true;
            }
            return false;
        }
        return false;
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
        return false; //TODO
    }
    public boolean getIntakeRotatorDown() {
        return false; //TODO
    }

    //Endgame
    public boolean getEndgameFlipOut() {
        return false; //TODO
    }
    public boolean getEndgameFlipIn() {
        return false; //TODO
    }
}