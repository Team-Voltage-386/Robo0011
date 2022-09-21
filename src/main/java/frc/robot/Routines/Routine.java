package frc.robot.Routines;

/** 
 * the super class for any routine
 * @author Carl C
 */
public class Routine {
    /** the order with which routines will be executed, 0 being first */
    public int execOrder = 0;
    /** public value for checking state */
    public boolean running = false;
    /** give the routine a title to make debug easier */
    public String info = "";

    /** ran when the routine is added to the schedule */
    public void begin() {

    }

    /** ran every cycle */
    public void exec() {

    }

    /** ran when the scheduler removes the routine from the schedule */
    public void end() {

    }

    /** used for sequential routines NOT IMPLEMENTED YET */
    public boolean finished() {
        return false;
    }

    /** use at any time to print the info usually from an array of routines */
    public void printInfo() {
        System.out.println(info);
    }
}
