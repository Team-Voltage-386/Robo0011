package frc.robot;

/** The Utils class contains general purpose methods, and the flags class */
public class Utils {
    /** Static flags to allow subsystems and other non-static classes to pass around info, because the wpi robot template is way too object-oriented for its own good */
    public static class Flags {

        // these flags allow the subsystems in the D_TeleOp command to communicate with the M_TeleOp command
        /** communicates whether or not the robot is lined up for the shot */
        public static boolean hoopLocked = false;
        /** communicates the distance to the target */
        public static double targetDistance = 3;
        /** allows the driver to override the compliance in the manipulator's code */
        public static boolean complianceOverride = false;
        /** communicates whether or not the target is visible */
        public static boolean hoopVisible = false;

        // these flags allow the subsystems in M_TeleOp communicate to the D_TeleOp command
        /** communicates whether or not the targeting sequence is active */
        public static boolean hoopTargeted = false;
    }

    /**Lerp A, Standard Linear Interpolation
     * @param a first number
     * @param b second number
     * @param factor lerp factor
     */
    public static double lerpA(double a, double b, double factor) {
        double c = b-a;
        c = factor * c;
        return a + c;
    }

    /** Lerp B, Absolute Value Linear Interpolation
     * @param a first number
     * @param b second number
     * @param factor lerp factor
     */
    public static double lerpB(double a, double b, double factor) {
        double c = Math.abs(b - a);
        c = factor * c;
        return a + c;
    }

    public static interface doubAlg {
        double get(double a);
    }

    public static interface doubAlgB {
        double get(double a, double b);
    }
}
