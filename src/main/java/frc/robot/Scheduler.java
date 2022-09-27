package frc.robot;

import java.util.Arrays;

import frc.robot.Routines.Routine;

/** The routine scheduler class
 * @author Carl C
 */
public class Scheduler {
    private static Routine[] routineList = new Routine[0];

    /** add a group of routines to the active list */
    public static void addRoutines(Routine[] ar) {
        for (Routine r : ar) {
            boolean skip = false;
            for (Routine existing : routineList)
                skip = r == existing;
            if (!skip) {
                routineList = addRoutineArray(routineList, r);
                r.begin();
            }
        }

        printActiveRoutines(); // print active routines for debug
    }

    /** remove a group of routines from the active list */
    public static void removeRoutines(Routine[] ar) {
        for (Routine r : ar) {
            int rem = 0;
            boolean skip = true;
            while (rem < ar.length) {
                if (ar[rem].equals(r)) {
                    skip = false;
                    break;
                } else
                    rem++;
            }
            if (!skip) {
                removeRoutineAtIndex(rem);
            }
        }

        printActiveRoutines(); // print active routines for debug
    }

    /** removes the specified routine from the list
     * @param rem index at which to remove routine
     */
    private static void removeRoutineAtIndex(int rem) {
        routineList[rem].end();
        Routine[] res = new Routine[routineList.length - 1];
        Routine[] low = new Routine[0];
        if (rem > 0) low = Arrays.copyOfRange(routineList, 0, rem - 1);
        Routine[] high = new Routine[0];
        if (rem < routineList.length-1) Arrays.copyOfRange(routineList, rem + 1, routineList.length - 1);
        
        for (int i = 0; i < routineList.length - 1; i++) {
            if (i < rem)
                res[i] = low[i];
            else
                res[i] = high[i - rem];
        }
        routineList = res;
    }

    /** clears the active routine list <p>NOT SAFE */
    public static void empty() {
        routineList = new Routine[0];
    }

    /** executes the exec() function in every routine active */
    public static void execute() {
        for (int i = 0; i < routineList.length; i++) {
            if (routineList[i].finished()) removeRoutineAtIndex(i);
            else routineList[i].exec();
        }
    }

    /**
     * adds routines and keeps them sorted by their execution order
     * 
     * @param ar array of routines to add to
     * @param r  routine to add
     * @return routine array with new routine inserted in execution order
     */
    public static Routine[] addRoutineArray(Routine[] ar, Routine r) {
        Routine[] res = new Routine[ar.length + 1];
        if (ar.length > 0) { // if not adding first routine
            int insInd = 0;
            while (insInd < ar.length || r.execOrder > ar[insInd].execOrder) // step through to find where it belongs in
                                                                             // the execution order
                insInd++;
            Routine[] low = Arrays.copyOfRange(ar, 0, insInd - 1); // array copying here
            Routine[] high = Arrays.copyOfRange(ar, insInd, ar.length - 1);
            for (int i = 0; i < res.length; i++) {
                if (i < insInd)
                    res[i] = low[i];
                else if (i == insInd)
                    res[i] = r;
                else
                    res[i] = high[(i - insInd) - 1];
            }
        } else { // if adding first routine simply add it and return
            res[0] = r;
        }
        return res;
    }

    public static void printActiveRoutines() {
        System.out.println("ACTIVE ROUTINES: ");
        for (Routine r : routineList)
            r.printInfo();
    }

}
