package frc.robot.Routines;

import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Utils;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Launcher;
import frc.robot.Subsystems.LimeLight;

public class DriverRoutine extends Routine {

    /** variable to hold instance or "active" object */
    public static DriverRoutine inst;

    private boolean highGear = false;
    private double finalDrive = 0;
    private double finalTurn = 0;
    private double integralTurnAdjust = 0;
    private double lastTurn = 0;


    public DriverRoutine() {
        // instance this code to make sure it only exists once
        if (inst == null) inst = this;

        this.execOrder = 1; // set execution order
        this.info = "Driver Routine: handles driver input"; // give title
    }

    @Override
    public void begin() {
        System.out.println("Driver Routine Starting");
        Drivetrain.setHighGear(false);
        highGear = false;
        finalDrive = 0;
        finalTurn = 0;
        integralTurnAdjust = 0;
        lastTurn = 0;
        running = true; // be sure to set running true
    }

    @Override
    public void exec() {
        // get driver input
        double contDrive = kDriverController.getRawAxis(kLeftVertical);
        double contTurn = -kDriverController.getRawAxis(kRightHorizontal);

        if (Drivetrain.highGear) {
            contDrive *= highGearInputLimit;
            contDrive -= (1 - highGearInputLimit) * kDriverController.getRawAxis(kRightTrigger);
        }
        if (Math.abs(contDrive) > Math.abs(finalDrive)) finalDrive = Utils.lerpA(finalDrive, contDrive, kSmoothingAccelFactor);
        else finalDrive = Utils.lerpA(finalDrive, contDrive, kSmoothingDecelFactor);

        // turn behavior uses a feedforward system
        integralTurnAdjust += contTurn - lastTurn;
        finalTurn = contTurn;
        if (Drivetrain.highGear) finalTurn += (2*integralTurnAdjust);
        lastTurn = contTurn;
        if (Drivetrain.highGear) finalTurn *= highGearTurnLimit;
        integralTurnAdjust *= kIntegralTurnDecay;

        // set high gear
        if (kDriverController.getRawButtonPressed(kLeftBumper))
            highGear = !highGear;
        Drivetrain.setHighGear(highGear);

        // handle targeting behavior
        if (kManipulatorController.getRawAxis(kRightTrigger) > 0.5 && LimeLight.targetFound) {
            if (Math.abs(LimeLight.tx) < 0.9) {
                ltPID.reset();
                finalTurn = 0;
            } else finalTurn = ltALG.get(LimeLight.tx);
            if (Math.abs(LimeLight.tx) < 1.2) Launcher.robotAligned = true;
        } else  {
            Launcher.robotAligned = false;
            ltPID.reset();
        }
        // output final drive
        Drivetrain.arcadeDrive(finalDrive, finalTurn);
    }

    @Override
    public void end() {
        System.out.println("Driver Routine Ending");
        running = false; // be sure to set runnning false
        Launcher.robotAligned = false;
    }
}
