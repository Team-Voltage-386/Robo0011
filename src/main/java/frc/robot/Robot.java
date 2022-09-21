// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Routines.DriverRoutine;
import frc.robot.Routines.Routine;
import frc.robot.Subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class. If you change the name of this class or the
 * package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends RobotBase {

  private int mode = 0;
  private Routine[] disabledRoutines = new Routine[0];
  private Routine[] autonomousRoutines = new Routine[0];
  private Routine[] teleopRoutines = new Routine[0];
  private Routine[] enabledRoutines = new Routine[0];


  public void robotInit() {
    // init subsystems here
    Drivetrain.init();

    // instance routines here
    DriverRoutine.inst();

    // declare routine groups here
    teleopRoutines = Scheduler.addRoutineArray(teleopRoutines, DriverRoutine.act);
  }
  public void robotPeriodic() {
    // subsystems update
    Drivetrain.update();

    // custom code
    
  }

  public void disabledInit() {
    Scheduler.removeRoutines(autonomousRoutines);
    Scheduler.removeRoutines(teleopRoutines);
    Scheduler.removeRoutines(enabledRoutines);
    Scheduler.addRoutines(disabledRoutines);
    // custom init code after scheduler calls

  }
  public void disabledPeriodic() {
    // custom code here
    
  }

  public void autonomousInit() {
    Scheduler.removeRoutines(disabledRoutines);
    Scheduler.removeRoutines(teleopRoutines);
    Scheduler.addRoutines(enabledRoutines);
    Scheduler.addRoutines(autonomousRoutines);
    // custom init code after scheduler calls
    
  }
  public void autonomousPeriodic() {
    // custom code here
    
  }

  public void teleopInit() {
    Scheduler.removeRoutines(disabledRoutines);
    Scheduler.removeRoutines(autonomousRoutines);
    Scheduler.addRoutines(enabledRoutines);
    Scheduler.addRoutines(teleopRoutines);
    // custom init code after scheduler calls
    
  }
  public void teleopPeriodic() {
    // custom code here

  }

  private volatile boolean m_exit;
  @Override
  public void startCompetition() {
    robotInit();

    // Tell the DS that the robot is ready to be enabled
    HAL.observeUserProgramStarting();

    while (!Thread.currentThread().isInterrupted() && !m_exit) {
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
      robotPeriodic();
      Scheduler.execute();
    }
  }

  @Override
  public void endCompetition() {
    m_exit = true;
  }
}
