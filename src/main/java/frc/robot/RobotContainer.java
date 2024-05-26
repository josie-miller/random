// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.commands.RunIntake;
import frc.robot.commands.MoveElevatorToPosition;

public class RobotContainer {
    private final Intake intake = new Intake();
    private final Elevator elevator = new Elevator();
    private final XboxController operatorController = new XboxController(1);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Bind button A to run intake
        new Trigger(() -> operatorController.getAButton())
            .whileTrue(new RunIntake(intake, 4.0));

        // Bind button B to reverse intake
        new Trigger(() -> operatorController.getBButton())
            .whileTrue(new RunIntake(intake, -4.0));

        // Bind button X to move elevator to position 1
        new Trigger(() -> operatorController.getXButton())
            .whenActive(new MoveElevatorToPosition(elevator, 10.0));

        // Bind button Y to move elevator to position 2
        new Trigger(() -> operatorController.getYButton())
            .whenActive(new MoveElevatorToPosition(elevator, 20.0));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
