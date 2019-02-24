package frc.robot.util.pid;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PIDTuner {
    private PIDTunable controller;
    private PIDValue defaultValues;
    private ShuffleboardTab manualPIDFTuner;
    private NetworkTableEntry networkTableKP, networkTableKI, networkTableKD, networkTableKF;
    private NetworkTableEntry inputValue;

    public PIDTuner(PIDTunable controller) {
        this(controller, new PIDValue(0,0,0,0));
    }
    public PIDTuner(PIDTunable controller, PIDValue defaultValues) {
        this.controller = controller;
        this.defaultValues = new PIDValue(defaultValues);
        manualPIDFTuner = Shuffleboard.getTab("PIDF Tuner");
        networkTableKP = manualPIDFTuner.add("KP", defaultValues.getKP()).getEntry();
        networkTableKI = manualPIDFTuner.add("KI", defaultValues.getKI()).getEntry();
        networkTableKD = manualPIDFTuner.add("KD", defaultValues.getKD()).getEntry();
        networkTableKF = manualPIDFTuner.add("KF", defaultValues.getKF()).getEntry();
        inputValue = manualPIDFTuner.add("PID Output Value", 0.0).getEntry();
    }

    public void refreshPIDValues() {
        PIDValue newValues = new PIDValue(networkTableKP.getDouble(defaultValues.getKP()), networkTableKP.getDouble(defaultValues.getKI()), 
            networkTableKP.getDouble(defaultValues.getKD()), networkTableKP.getDouble(defaultValues.getKF()));
        controller.setPIDFConstants(newValues);
        defaultValues = new PIDValue(newValues);
        inputValue.setDouble(controller.getPIDOutput());
    }
}