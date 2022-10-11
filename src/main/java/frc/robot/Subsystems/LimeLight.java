package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.LimeLightConstants;
import static frc.robot.Constants.LimeLightConstants.*;

public class LimeLight {
    /** The networkTable instance for the camera */
    private static NetworkTable llTable;

    /** Whether or not the LL should wait before declaring target loss to see if it will come back */
    public static boolean targetLostWait = true;
    public static boolean dMode = false;
    public static boolean targetFound = false;
    public static boolean targetLocked = false;
    /** target right/left angle */
    public static float tx = 0;
    /** target up/down angle */
    public static float ty = 0;
    private static final Timer timer = new Timer();

    public static void init() {
        llTable = NetworkTableInstance.getDefault().getTable("limelight");
        timer.start();
        driverMode(false);
        setPipeLine(kDefaultPipeline);
        targetFound = false;
        targetLocked = false;
    }

    public static void update() {
        if(llTable.getEntry("tv").getDouble(-1) == 0) { // that '-1' saved our season, a random choice to change the default to anything other than zero
            if(targetLostWait) targetFound = !timer.hasElapsed(LimeLightConstants.targetLostWaitTime); // if it should wait for target re-acquire, then wait the tlwt, else declare it lost
            else targetFound = false;
        } else {
            targetFound = true; // if target is found, update values and keep timer reset, but running
            timer.reset();
            timer.start();
            tx = (float)llTable.getEntry("tx").getDouble(0);
            ty = (float)llTable.getEntry("ty").getDouble(0);
            targetLocked = Math.abs(tx) < kAimTolerance;
        }
    }

    /**Returns the meters to the target given the target's height from the ground*/
    public static double metersToTarget() {
        return distALG.get(ty);
    }

    /** Set drivermode (Exposure turned up to make image visible to mere humans)
     * @param b whether or not drivermode should be turned on or off
     */
    public static void driverMode(Boolean b) {
        dMode = b;
        if (b) llTable.getEntry("camMode").setNumber(1);
        else llTable.getEntry("camMode").setNumber(0);
    }

    /**
     * Set the LimeLight Pipeline
     * @param p Pipeline Index
     */
    public static void setPipeLine(Integer p) {
        llTable.getEntry("pipeline").setNumber(p);
    }

    /** turn lights on/off */
    public static void lights(boolean b) {
        if (b) llTable.getEntry("ledMode").setNumber(3);
        else llTable.getEntry("ledMode").setNumber(1);
    }
}
