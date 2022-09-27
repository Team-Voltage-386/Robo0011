package frc.robot.Routines;

import static frc.robot.Constants.ControllerConstants.*;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Subsystems.Drivetrain;

public class DriverRoutine extends Routine {

    /** variable to hold instance or "active" object */
    public static DriverRoutine act;
    
    private final Joystick controller;


    public DriverRoutine() {
        this.execOrder = 1; // set execution order
        this.info = "Routine that handles driver input"; // give title
        
        controller = driverController;
    }

    /** use in every routine that shouldn't have more than one running at once ever, is an instancer */
    public static void inst() {
        if (act == null) {
            act = new DriverRoutine();
        }
    }

    @Override
    public void begin() {
        System.out.println("Driver Routine Starting");
        Drivetrain.setHighGear(false);
        running = true; // be sure to set running true
    }

    @Override
    public void exec() {
        double contDrive = controller.getRawAxis(kLeftVertical); // very simple, just an example
        double contTurn = -controller.getRawAxis(kRightHorizontal);

        Drivetrain.arcadeDrive(contDrive, contTurn);
    }

    @Override
    public void end() {
        System.out.println("Driver Routine Ending");
        running = false; // be sure to set runnning false
    }
}
