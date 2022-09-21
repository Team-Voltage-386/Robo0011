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
import frc.robot.Routines.RoutineBase;
import frc.robot.Subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class. If you change the name of this class or the
 * package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends RobotBase {

  private int mode = 0;
  private RoutineBase[] disabledRoutines = new RoutineBase[0];
  private RoutineBase[] autonomousRoutines = new RoutineBase[0];
  private RoutineBase[] teleopRoutines = new RoutineBase[0];


  public void robotInit() {
    // init subsystems here
    Drivetrain.init();

    // instance routines here
    DriverRoutine.inst();

    // declare routine groups here
    teleopRoutines = {DriverRoutine.act};
  }
  public void robotPeriodic() {
    Drivetrain.update();
  }

  public void disabledInit() {}
  public void disabledPeriodic() {}

  public void autonomousInit() {}
  public void autonomousPeriodic() {}

  public void teleopInit() {}
  public void teleopPeriodic() {}

  public void test() {}



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
