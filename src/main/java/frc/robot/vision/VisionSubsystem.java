package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.inject.Singleton;

@Singleton
public class VisionSubsystem extends SubsystemBase {
    private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();

    private NetworkTableEntry getValue(String key) {
        return networkTable.getTable("limelight").getEntry(key);
    }

    public double getDistance() {
        double x_val = getValue("tx").getDouble(0);
        double y_val = getValue("tx").getDouble(0);

        return Math.sqrt(x_val * x_val + y_val * y_val);
    }
}
