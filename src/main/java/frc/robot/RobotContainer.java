package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.handoff.Handoff;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.ElevatorOuttakeSequence;
import frc.robot.commands.MoveElevatorToPosition;
import frc.robot.commands.RunHandoff;

public class RobotContainer {
    private final Intake intake = new Intake();
    private final Elevator elevator = new Elevator();
    private final Handoff handoff = new Handoff();
    private final Shooter shooter = new Shooter();
    public static final CommandXboxController operatorController = new CommandXboxController(0);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        operatorController.a()
            .onTrue(new RunIntake(intake, 4.0, handoff));

        operatorController.x()
            .onTrue(new ElevatorOuttakeSequence(elevator, intake));

        operatorController.y() 
            .whileTrue(new RunShooter(shooter,20,0.7)); 
        operatorController.b()  
            .whileTrue(new RunHandoff(handoff, 5));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
