package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limelightInterface {
    private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    public static double getDoubleEnrty(String entry) {
        return limelight.getEntry(entry).getDouble(0);
    }

    public static double[] getArrayEntry(String entry) {
        return limelight.getEntry(entry).getDoubleArray(new double[6]);
    }

    public static double getID() {
        return getDoubleEnrty("tid");
    }

    public static double getTargetArea() {
        return getDoubleEnrty("ta");
    }

    public static boolean hasValidTarget() {
        return getDoubleEnrty("tv") == 1.0;
    }

    public static double getXOffset() {
        return getDoubleEnrty("tx");
    }
/** Y offset in degrees. */
    public static double getYOffset() {
        return getDoubleEnrty("ty");
    }

    public static double[] getArrayEntry() {
        return getArrayEntry("botpose_targetspace");
    }

    public static double distance = Math.sqrt(getXOffset() * getXOffset() + getYOffset() * getYOffset());

    public static double autoAngle = Math.atan((getYOffset() + 11));
}
