package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.commands.RunIntake;
import frc.robot.commands.MoveElevatorToPosition;

public class RobotContainer {
    private final Intake intake = new Intake();
    private final Elevator elevator = new Elevator();
    public static final CommandXboxController operatorController = new CommandXboxController(0);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        operatorController.a()
            .whileTrue(new RunIntake(intake, 4.0));

        operatorController.b()
            .whileTrue(new RunIntake(intake, -4.0));

        operatorController.x()
            .onTrue(new MoveElevatorToPosition(elevator, 0.1));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
