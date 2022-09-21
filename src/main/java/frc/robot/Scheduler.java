package frc.robot;

import frc.robot.Routines.RoutineBase;

public class Scheduler {
    private static RoutineBase[] routineList = new RoutineBase[0];

    public static void addRoutines(RoutineBase[] r) {

    }

    public static void removeRoutines(RoutineBase[] r) {
        
    }

    public static void halt() {
        routineList = new RoutineBase[0];
    }

    public static void execute() {

    }

}
