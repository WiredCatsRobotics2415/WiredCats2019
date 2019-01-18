package frc.robot.util;

public class DriveSignal {
    private double rightSignal;
    private double leftSignal;

    public DriveSignal(double speed, double r) {
        rightSignal = speed - r;
        leftSignal = speed + r;
    }

    public DriveSignal(double speed, double r, boolean quickTurn) {
        rightSignal = -r;
        leftSignal = r;
    }

    public double getRight() {
        return rightSignal;
    }

    public double getLeft() {
        return leftSignal;
    }
}