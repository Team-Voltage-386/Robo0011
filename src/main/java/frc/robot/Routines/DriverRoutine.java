package frc.robot.Routines;

import static frc.robot.Constants.ControllerConstants.*;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Subsystems.Drivetrain;

public class DriverRoutine extends Routine {

    public static DriverRoutine act;
    
    private final Joystick controller;


    public DriverRoutine() {
        this.execOrder = 1;
        this.info = "Routine that handles driver input";
        controller = driverController;
    }

    public static void inst() {
        if (act == null) {
            act = new DriverRoutine();
        }
    }

    @Override
    public void begin() {
        System.out.println("Driver Routine Starting");
        running = true;
    }

    @Override
    public void exec() {
        double contDrive = controller.getRawAxis(kLeftVertical);
        double contTurn = controller.getRawAxis(kRightHorizontal);

        Drivetrain.arcadeDrive(contDrive, contTurn);
    }

    @Override
    public void end() {
        System.out.println("Driver Routine Ending");
        running = false;
    }
}
