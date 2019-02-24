package frc.robot.util.pid;

public class PIDValue {
    private double kP, kI, kD, kF;

    public PIDValue(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }
    //Copy Constructor
    public PIDValue(PIDValue originalCopy) {
        this.kP = originalCopy.getKP();
        this.kI = originalCopy.getKI();
        this.kD = originalCopy.getKD();
        this.kF = originalCopy.getKF();
    }

    public double getKP() {
        return kP;
    }

    public double getKI() {
        return kI;
    }

    public double getKD() {
        return kD;
    }

    public double getKF() {
        return kF;
    }

    public void setKP(double kP) {
        this.kP = kP;
    } 

    public void setKI(double kI) {
        this.kI = kI;
    } 

    public void setKD(double kD) {
        this.kD = kD;
    } 

    public void setKF(double kF) {
        this.kF = kF;
    } 
}