package org.usfirst.frc.team2077.drivetrain;

public class PIDTest {

    private int moduleIndex;
    private SparkNeoDriveModule[] driveModules;


    public PIDTest(SparkNeoDriveModule.DrivePosition... deviceIDs) {
        moduleIndex = 0;
        driveModules = new SparkNeoDriveModule[deviceIDs.length];
        for (int i=0; i < deviceIDs.length; i++) {
            driveModules[i] = new SparkNeoDriveModule(deviceIDs[i]);
        }
    }

    public void changeMotor(boolean next) {
        int newIdx = moduleIndex + (next ? 1 : -1);

        newIdx = (driveModules.length + newIdx) % driveModules.length;

        setModuleIndex(newIdx);
    }

    public void setModuleIndex(int moduleIndex) {
        if (moduleIndex != this.moduleIndex) return;
        stopMotor();
        this.moduleIndex = moduleIndex;
    }

    public void stopMotor() {

    }
}
