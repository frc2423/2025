package frc.robot;

import java.util.EnumSet;
import java.util.function.Consumer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;

public class NTHelper {

    public static void setPersistent(String key) {
        getEntry(key).setPersistent();
    }

    /**
     * Adds an entry listener to network tables
     * 
     * @param key      String for network tables key
     * @param listener Function to be called when value changes
     */
    public static void listen(String key, Consumer<NetworkTableEvent> listener) {
        var entry = NTHelper.getEntry(key);
        NetworkTableInstance.getDefault().addListener(entry, EnumSet.of(Kind.kValueAll), listener);
    }

    /**
     * Get current value from network tables
     * 
     * @param key Key to get value of
     * @return current value of key
     */
    public static NetworkTableEntry getEntry(String key) {
        return NetworkTableInstance.getDefault().getEntry(key);
    }

    /**
     * Get current value from network tables
     * 
     * @param key          Key to get value of
     * @param defaultValue default value if key in network tables is null
     * @return current value of key
     */
    public static double getDouble(String key, double defaultValue) {
        return getEntry(key).getDouble(defaultValue);
    }

    /**
     * Sets the current value to network tables
     * 
     * @param key   key to set
     * @param value new value for key
     */
    public static void setDouble(String key, double value) {
        getEntry(key).setDouble(value);
    }

    /**
     * Get current value from network tables
     * 
     * @param key          Key to get value of
     * @param defaultValue default value if key in network tables is null
     * @return current value of key
     */
    public static String getString(String key, String defaultValue) {
        return getEntry(key).getString(defaultValue);
    }

    /**
     * Sets the current value to network tables
     * 
     * @param key   key to set
     * @param value new value for key
     */
    public static void setString(String key, String value) {
        getEntry(key).setString(value);
    }

    /**
     * Sets the current value to network tables
     * 
     * @param key   key to set
     * @param value new value for key
     */
    public static void setBoolean(String key, Boolean value) {
        getEntry(key).setBoolean(value);
    }

    /**
     * Get current value from network tables
     * 
     * @param key          Key to get value of
     * @param defaultValue default value if key in network tables is null
     * @return current value of key
     */
    public static boolean getBoolean(String key, Boolean defaultValue) {
        return getEntry(key).getBoolean(defaultValue);
    }

    /**
     * Sets the current value to network tables
     * 
     * @param key   key to set
     * @param value new value for key
     */
    public static void setStringArray(String key, String[] value) {
        getEntry(key).setStringArray(value);
    }

    /**
     * Sets the current value to network tables
     * 
     * @param key   key to set
     * @param value new value for key
     */
    public static void setDoubleArray(String key, double[] value) {
        getEntry(key).setDoubleArray(value);
    }

    public static double[] getDoubleArrayPose3d(Pose3d pose) {
        var trans = pose.getTranslation();
        var rot = pose.getRotation().getQuaternion();
        return new double[] { trans.getX(), trans.getY(), trans.getZ(), rot.getW(), rot.getX(),
                rot.getY(), rot.getZ() };
    }
    
    public static double[] getDoubleArrayPose2d(Pose2d pose) {
        var trans = pose.getTranslation();
        var rot = pose.getRotation();
        return new double[] { trans.getX(), trans.getY(), rot.getDegrees() };
    }
}