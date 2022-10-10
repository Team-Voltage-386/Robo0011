package frc.robot.Routines;

import frc.robot.Subsystems.Launcher;
import static frc.robot.Constants.ControllerConstants.*;

public class ManipulatorRoutine extends Routine {
    public static ManipulatorRoutine inst;
    private boolean climbActive = false;
    private boolean sentUp = false;

    public ManipulatorRoutine() {
        if (inst == null) inst = this;

        this.execOrder = 2; // set execution order
        this.info = "Manipulator Routine: handles manipulator input"; // add title
    }

    @Override
    public void begin() {
        System.out.println("Manipulator Routine Starting");
        //_bss.reset();
        climbActive = false;
        Launcher.drumIdle = false;
        sentUp = true; // I thought this was set false at init? idk why it works if this is true
        //_bss.intakeUpdate(!_bss.intakeOut);
    }

    @Override
    public void exec() {
        // Control Logic
    if(kManipulatorController.getRawButtonPressed(kA)) Launcher.drumIdle = Launcher.drumIdle; // toggle drum idle
    Launcher.intakeUpdate(kManipulatorController.getRawButtonPressed(kRightBumper)); // deploy/retract/release intake
    if (kManipulatorController.getRawButtonPressed(kLeftBumper)) Launcher.lowShot = !Launcher.lowShot;
    if (triggerReleased(kRightTrigger)) _bss.afterFiring();
    if (triggerPressed(kRightTrigger)) _bss.fireTheBigIron = true; // begin targeting if rt is pressed
    hoopTargeted = _bss.fireTheBigIron;

    if (hoopTargeted) _bss.setAimDistance(targetDistance);

    // elevator logic
    if (climbActive) {
      if (sentUp) _kss.setElePower(-0.85* Math.pow(_controller.getRawAxis(kRightVertical),3));
      else {
        if (!(_kss.getEnc().getPosition() > 35)) _kss.setElePower(0.8);
        else {
          sentUp = true;
          _kss.setElePower(0);
        }
      }
    } else {
      _kss.setElePower(0);
      sentUp = false;
    }

    if (_controller.getRawButtonPressed(kB)) _bss.decreaseBC();
    if (_controller.getRawButtonPressed(kY)) _bss.increaseBC();

    if (_controller.getRawButtonPressed(kX)) climbActive = !climbActive;
    if (_controller.getRawButtonPressed(kRightJoystickPressed)) _bss.reset();
    _bss.climbing = climbActive;
    }

    @Override 
    public void end() {
        System.out.println("Manipulator Routine Ending");
    }
}
