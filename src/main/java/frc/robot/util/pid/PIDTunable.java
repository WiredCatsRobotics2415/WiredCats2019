package frc.robot.util.pid;

public interface PIDTunable {
    public abstract void setPIDFConstants(PIDValue[] newValues);
    public abstract double[] getPIDOutput();
}