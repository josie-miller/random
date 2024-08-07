// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autons.AutonomousSelector.modes;
import frc.robot.autons.modes.ChoreoTest;
import frc.robot.autons.modes.PreloadAmp;
import frc.robot.autons.modes.PreloadMid;
import frc.robot.autons.modes.PreloadSource;
import frc.robot.autons.modes.TwoPieceAmp;
import frc.robot.autons.modes.TwoPieceMid;
import frc.robot.autons.modes.TwoPieceSource;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  SequentialCommandGroup choreo_test;
  SequentialCommandGroup two_piece_amp;
  SequentialCommandGroup two_piece_mid;
  SequentialCommandGroup two_piece_source;
  SequentialCommandGroup preload_amp;
  SequentialCommandGroup preload_mid;
  SequentialCommandGroup preload_source;

  private boolean built = false;

  @Override
  public void robotInit() {
      SignalLogger.setPath("/media/sda1/");
    
      Logger.recordMetadata("Marmalade", "RobotLogs");

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
      new PowerDistribution(1, ModuleType.kRev);
    } else {
      setUseTiming(false); 
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }
    Logger.start();
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
      if (DriverStation.getAlliance().isPresent() && !built){
        choreo_test = new ChoreoTest(m_robotContainer.getSwerve(), m_robotContainer.getIntake(), m_robotContainer.getOtbIntake(), m_robotContainer.getHandoff(), m_robotContainer.getShooter());
        two_piece_amp = new TwoPieceAmp(m_robotContainer.getSwerve(), m_robotContainer.getIntake(), m_robotContainer.getOtbIntake(), m_robotContainer.getHandoff(), m_robotContainer.getShooter());
        two_piece_mid = new TwoPieceMid(m_robotContainer.getSwerve(), m_robotContainer.getIntake(), m_robotContainer.getOtbIntake(), m_robotContainer.getHandoff(), m_robotContainer.getShooter());
        two_piece_source = new TwoPieceSource(m_robotContainer.getSwerve(), m_robotContainer.getIntake(), m_robotContainer.getOtbIntake(), m_robotContainer.getHandoff(), m_robotContainer.getShooter());
        preload_amp = new PreloadAmp(m_robotContainer.getSwerve(), m_robotContainer.getIntake(), m_robotContainer.getOtbIntake(), m_robotContainer.getHandoff(), m_robotContainer.getShooter());
        preload_mid = new PreloadMid(m_robotContainer.getSwerve(), m_robotContainer.getIntake(), m_robotContainer.getOtbIntake(), m_robotContainer.getHandoff(), m_robotContainer.getShooter());
        preload_source = new PreloadSource(m_robotContainer.getSwerve(), m_robotContainer.getIntake(), m_robotContainer.getOtbIntake(), m_robotContainer.getHandoff(), m_robotContainer.getShooter());
      built = true;

      }

  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    if (m_robotContainer.getAutonomousCommand() == modes.CHOREOTEST){
      m_autonomousCommand = choreo_test;
    }
    else if (m_robotContainer.getAutonomousCommand() == modes.TWOPIECEAMP){
      m_autonomousCommand = two_piece_amp;
    }
    else if (m_robotContainer.getAutonomousCommand() == modes.TWOPIECEMID){
      m_autonomousCommand = two_piece_mid;
    }
    else if (m_robotContainer.getAutonomousCommand() == modes.TWOPIECESOURCE){
      m_autonomousCommand = two_piece_source;
    }
    else if (m_robotContainer.getAutonomousCommand() == modes.PRELOADAMP){
      m_autonomousCommand = preload_amp;
    }
    else if (m_robotContainer.getAutonomousCommand() == modes.PRELOADMID){
      m_autonomousCommand = preload_mid;
    }
    else if (m_robotContainer.getAutonomousCommand() == modes.PRELOADSOURCE){
      m_autonomousCommand = preload_source;
    }
    else if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
