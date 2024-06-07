package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.handoff.Handoff;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.otb_intake.otbIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.commands.ShootSequence;
import frc.robot.commands.AMPShoot;
import frc.robot.commands.ElevatorOuttakeSequence;
import frc.robot.commands.IntakeSequence;

public class RobotContainer {
    private final Intake intake = new Intake();
    private final Elevator elevator = new Elevator();
    private final Handoff handoff = new Handoff();
    private final Shooter shooter = new Shooter();
    private final otbIntake otbIntake = new otbIntake();
    public static final CommandXboxController operatorController = new CommandXboxController(0);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        operatorController.a()
            .onTrue(new IntakeSequence(otbIntake, intake, handoff));

        operatorController.x()
            .onTrue(new ElevatorOuttakeSequence(elevator, intake));
//mid shoot
        operatorController.y() 
            .onTrue(new ShootSequence(intake, handoff, shooter, Constants.commandConstants.midShootTime, Constants.commandConstants.midShootRatio, Constants.commandConstants.handoffIntakeVoltage, Constants.commandConstants.handoffShooterVoltage, Constants.commandConstants.shootMidVelocity, Constants.commandConstants.midShortTime)); 
//amp shoot
        operatorController.b()  
            .onTrue(new AMPShoot(intake, handoff, shooter));
            //left shoot
            //.onTrue(new ShootSequence(intake, handoff, shooter, Constants.commandConstants.leftShootTime, Constants.commandConstants.leftShootRatio, Constants.commandConstants.handoffIntakeVoltage, Constants.commandConstants.handoffShooterVoltage, Constants.commandConstants.shootLeftVelocity, Constants.commandConstants.leftShortTime)); 
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
