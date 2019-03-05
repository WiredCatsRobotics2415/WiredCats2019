package frc.robot.util.pid;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PIDTuner {
    private PIDTunable controller;
    private PIDValue[] defaultValues;
    private ShuffleboardTab manualPIDFTuner;
    private NetworkTableEntry[][] networkTableEntries;
    private NetworkTableEntry[] inputValue;
    private int pidControllers;

    public PIDTuner(PIDTunable controller) {
        this(controller, new PIDValue(0,0,0,0));
    }

    public PIDTuner(PIDTunable controller, PIDValue defaultValues) {
        this(controller, null, 1);
        this.defaultValues[0] = new PIDValue(defaultValues);
        networkTableEntries[0][0].setDouble(this.defaultValues[0].getKP());
        networkTableEntries[0][1].setDouble(this.defaultValues[0].getKI());
        networkTableEntries[0][2].setDouble(this.defaultValues[0].getKD());
        networkTableEntries[0][3].setDouble(this.defaultValues[0].getKF());
    }

    public PIDTuner(PIDTunable controller, PIDValue[] defaultValues, int pidControllers) {
        this.controller = controller;
        this.pidControllers = pidControllers;
        this.defaultValues = new PIDValue[pidControllers];
        for(int i = 0; i < pidControllers; i++) {
            if(defaultValues != null && i < defaultValues.length) {
                this.defaultValues[i] = new PIDValue(defaultValues[i]);
            } else {
                this.defaultValues[i] = new PIDValue(0,0,0,0);
            }
        }
        networkTableEntries = new NetworkTableEntry[pidControllers][4];
        inputValue = new NetworkTableEntry[pidControllers];
        manualPIDFTuner = Shuffleboard.getTab("PIDF Tuner");
        for(int i = 0; i < pidControllers; i++) {
            networkTableEntries[i][0] = manualPIDFTuner.add("PID "+i+" KP", this.defaultValues[i].getKP()).getEntry();
            networkTableEntries[i][1] = manualPIDFTuner.add("PID "+i+" KI", this.defaultValues[i].getKI()).getEntry();
            networkTableEntries[i][2] = manualPIDFTuner.add("PID "+i+" KD", this.defaultValues[i].getKD()).getEntry();
            networkTableEntries[i][3] = manualPIDFTuner.add("PID "+i+" KF", this.defaultValues[i].getKF()).getEntry();
            inputValue[i] = manualPIDFTuner.add("PID "+i+" Output Value", controller.getPIDOutput()[i]).getEntry();
        }
    }

    public void refreshPIDValues() {
        PIDValue[] newValues = new PIDValue[pidControllers];
        for(int i = 0; i < newValues.length; i++) {
            newValues[i] = new PIDValue(networkTableEntries[i][0].getDouble(defaultValues[i].getKP()), networkTableEntries[i][1].getDouble(defaultValues[i].getKI()),
            networkTableEntries[i][2].getDouble(defaultValues[i].getKD()), networkTableEntries[i][3].getDouble(defaultValues[i].getKF()));
        }
        controller.setPIDFConstants(newValues);
        for(int i = 0; i < defaultValues.length; i++) {
            defaultValues[i] = new PIDValue(newValues[i]);
        }
        double[] outputs = controller.getPIDOutput();
        for(int i = 0; i < outputs.length && i < inputValue.length; i++) {
            inputValue[i].setDouble(outputs[i]);
        }
    }
}