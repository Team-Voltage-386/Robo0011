// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Routines.DriverRoutine;
import frc.robot.Routines.ManipulatorRoutine;
import frc.robot.Routines.Routine;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Launcher;
import frc.robot.Subsystems.LimeLight;

/**
 * The Routine-Based Main Robot Class
 * @author Carl C
 */
public class Robot extends RobotBase {

  private int mode = 0; // keeps track of last mode to operate init functions

  // the routine arrays for each mode
  private Routine[] disabledRoutines = new Routine[0];
  private Routine[] autonomousRoutines = new Routine[0];
  private Routine[] teleopRoutines = new Routine[0];
  private Routine[] enabledRoutines = new Routine[0];
  private Routine[] universalRoutines = new Routine[0];

  /** run on code start */
  public void robotInit() {
    // init subsystems here
    Drivetrain.init();
    LimeLight.init();
    Launcher.init();

    // instance routines here
    new DriverRoutine();

    // declare routine groups here
    teleopRoutines = Scheduler.addRoutineArray(teleopRoutines, DriverRoutine.inst); // add each new routine like this
    teleopRoutines = Scheduler.addRoutineArray(teleopRoutines, ManipulatorRoutine.inst);

    Scheduler.addRoutines(universalRoutines);
  }
  /** run every cycle */
  public void robotPeriodic() {
    // subsystems update
    Drivetrain.update();
    LimeLight.update();
    Launcher.update();

    // custom code
    
  }

  /** runs once when disabled */
  public void disabledInit() {
    Scheduler.removeRoutines(autonomousRoutines);
    Scheduler.removeRoutines(teleopRoutines);
    Scheduler.removeRoutines(enabledRoutines);
    Scheduler.addRoutines(disabledRoutines);
    // custom init code after scheduler calls

  }
  /** runs every cycle when disabled */
  public void disabledPeriodic() {
    // custom code here
    
  }

  /** runs once when auto is enabled */
  public void autonomousInit() {
    Scheduler.removeRoutines(disabledRoutines);
    Scheduler.removeRoutines(teleopRoutines);
    Scheduler.addRoutines(enabledRoutines);
    Scheduler.addRoutines(autonomousRoutines);
    // custom init code after scheduler calls
    
  }
  /** runs every cycle when auto is enabled */
  public void autonomousPeriodic() {
    // custom code here
    
  }

  /** runs once when teleop is enabled */
  public void teleopInit() {
    Scheduler.removeRoutines(disabledRoutines);
    Scheduler.removeRoutines(autonomousRoutines);
    Scheduler.addRoutines(enabledRoutines);
    Scheduler.addRoutines(teleopRoutines);
    // custom init code after scheduler calls
    
  }
  /** runs every cycle when teleop is enabled */
  public void teleopPeriodic() {
    // custom code here

  }

  private volatile boolean m_exit;
  @Override
  public void startCompetition() {
    robotInit();

    // Tell the DS that the robot is ready to be enabled
    HAL.observeUserProgramStarting();

    while (!Thread.currentThread().isInterrupted() && !m_exit) { // while the code has not errored out or been told to stop by DS keep going
      if (isDisabled()) {
        if (mode != 0) {
          disabledInit();
          mode = 0;
        }
        disabledPeriodic();
      } else if (isAutonomous()) {
        if (mode != 1) {
          autonomousInit();
          mode = 1;
        }
        autonomousPeriodic();
      } else if (isTeleop()) {
        if (mode != 2) {
          teleopInit();
          mode = 2;
        }
        teleopPeriodic();
      }
      robotPeriodic(); // run the periodic and the scheduler
      Scheduler.execute();
    }
  }

  @Override
  public void endCompetition() {
    m_exit = true;
  }
}
