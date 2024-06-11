package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.handoff.Handoff;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.otb_intake.otbIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.commands.ShootSequence;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.AMPShoot;
import frc.robot.commands.ElevatorOuttakeSequence;
import frc.robot.commands.IntakeSequence;

public class RobotContainer {
    private final Intake intake = new Intake();
    private final Elevator elevator = new Elevator();
    private final Handoff handoff = new Handoff();
    private final Shooter shooter = new Shooter();
    private final otbIntake otbIntake = new otbIntake();
    private final Swerve swerve = new Swerve();
    private static final CommandXboxController movementController = new CommandXboxController(0);
    public static final CommandXboxController operatorController = new CommandXboxController(1);

    public RobotContainer() {
    swerve.zeroWheels();
    swerve.zeroGyro();
    swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -movementController.getRawAxis(XboxController.Axis.kLeftY.value),
                () -> -movementController.getRawAxis(XboxController.Axis.kLeftX.value), 
                () -> -movementController.getRawAxis(XboxController.Axis.kRightX.value)
              
            )
        );
    
    configureButtonBindings();
    }

    private void configureButtonBindings() {
        operatorController.a()
            .onTrue(new IntakeSequence(otbIntake, intake, handoff));

        operatorController.x()
            .onTrue(new ElevatorOuttakeSequence(elevator, intake, otbIntake));
//mid shoot speaker
        operatorController.y() 
            .onTrue(new ShootSequence(intake, handoff, shooter, Constants.commandConstants.midShootTime, Constants.commandConstants.midShootRatio, Constants.commandConstants.handoffIntakeVoltage, Constants.commandConstants.handoffShooterVoltage, Constants.commandConstants.shootMidVelocity, Constants.commandConstants.midShortTime)); 
//shoot amp
        operatorController.b()  
            .onTrue(new AMPShoot(intake, handoff, shooter));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
