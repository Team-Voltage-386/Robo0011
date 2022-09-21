package frc.robot;

import java.util.Arrays;

import frc.robot.Routines.Routine;

public class Scheduler {
    private static Routine[] routineList = new Routine[0];

    public static void addRoutines(Routine[] ar) {
        for (Routine r : ar) {
            boolean skip = false;
            for (Routine existing : routineList) skip = r == existing;
            if (!skip) {
                routineList = addRoutineArray(routineList, r);
                r.begin();
            }
        }

        printActiveRoutines();
    }

    public static void removeRoutines(Routine[] ar) {
        for (Routine r : ar) {
            int rem = 0;
            boolean skip = true;
            while (rem < ar.length) {
                if (ar[rem].equals(r)) {
                    skip = false;
                    break;
                } else rem++;
            }
            if (!skip) {
                Routine[] res = new Routine[routineList.length-1];
                Routine[] low = Arrays.copyOfRange(routineList, 0, rem-1);
                Routine[] high = Arrays.copyOfRange(routineList, rem+1, ar.length-1);
                for (int i = 0; i < routineList.length-1; i++) {
                    if (i < rem) res[i] = low[i];
                    else res[i] = high[i-rem];
                }
                routineList = res;
                r.end();
            }
        }

        printActiveRoutines();
    }

    public static void empty() {
        routineList = new Routine[0];
    }

    public static void execute() {
        for (Routine r : routineList) r.exec();
    }

    /**
     * adds routines and keeps them sorted by their execution order
     * @param ar array of routines to add to
     * @param r routine to add
     * @return routine array with new routine inserted in execution order
     */
    public static Routine[] addRoutineArray(Routine[] ar, Routine r) {
        Routine[] res = new Routine[ar.length + 1];
        if (ar.length > 0) { // if not adding first routine
            int insInd = 0;
            while (insInd < ar.length || r.execOrder > ar[insInd].execOrder) // step through to find where it belongs in the execution order
                insInd++;
            Routine[] low = Arrays.copyOfRange(ar, 0, insInd - 1); // array copying here
            Routine[] high = Arrays.copyOfRange(ar, insInd, ar.length-1);
            for (int i = 0; i < res.length; i++) {
                if (i < insInd) res[i] = low[i];
                else if (i == insInd) res[i] = r;
                else res[i] = high[(i-insInd)-1];
            }
        } else { // if adding first routine simply add it and return
            res[0] = r;
        }
        return res;
    }

    public static void printActiveRoutines() {
        System.out.println("ACTIVE ROUTINES: ");
        for (Routine r : routineList) r.printInfo();
    }

}
