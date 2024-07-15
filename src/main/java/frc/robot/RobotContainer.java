package frc.robot;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.handoff.Handoff;
import frc.robot.subsystems.handoff.HandoffIOReal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.otb_intake.OtbIntake;
import frc.robot.subsystems.otb_intake.OtbIntakeIOReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.autons.AutonomousSelector;
import frc.robot.autons.AutonomousSelector.modes;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.sequences.ElevatorOuttakeSequence;
import frc.robot.commands.sequences.IntakeSequence;
import frc.robot.commands.sequences.ShootSequence;

import frc.robot.constants.commandConstants;

public class RobotContainer {
    private final Intake intake = new Intake(new IntakeIOReal());
    private final Handoff handoff = new Handoff(new HandoffIOReal());
    private final Elevator elevator = new Elevator(new ElevatorIOReal());
    private final Shooter shooter = new Shooter(new ShooterIOReal());
    private final OtbIntake otbIntake = new OtbIntake(new OtbIntakeIOReal());
    private final Swerve swerve = new Swerve();
    public static final CommandXboxController operatorController = new CommandXboxController(0);
    private AutonomousSelector selector;

    ChoreoTrajectory traj;

    public RobotContainer() {
    swerve.zeroWheels();
    swerve.zeroGyro();
    swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -operatorController.getRawAxis(XboxController.Axis.kLeftY.value),
                () -> -operatorController.getRawAxis(XboxController.Axis.kLeftX.value), 
                () -> -operatorController.getRawAxis(XboxController.Axis.kRightX.value)
              
            )
        );
    
    configureButtonBindings();
    }

    private void configureButtonBindings() {
        operatorController.a()
            .onTrue(new IntakeSequence(otbIntake, intake, handoff));

        operatorController.x() // Elevator
            .onTrue(new ElevatorOuttakeSequence(elevator, intake, otbIntake));

        operatorController.y() //AMP Shoot
            .onTrue(new ShootSequence(intake, handoff, shooter, commandConstants.AMPShootTime, commandConstants.AMPShootRatio, commandConstants.handoffIntakeVoltage, commandConstants.handoffShooterVoltage, commandConstants.AMPShooterVelocity, commandConstants.AMPShortTime));

        operatorController.b() //Speaker Mid Shoot
            .onTrue(new ShootSequence(intake, handoff, shooter, commandConstants.midShootTime, commandConstants.midShootRatio, commandConstants.handoffIntakeVoltage, commandConstants.handoffShooterVoltage, commandConstants.shootMidVelocity, commandConstants.midShortTime));
    }

    public modes getAutonomousCommand() {
        return selector.get();
    }

    public void configureAutonomousSelector(){
    selector = new AutonomousSelector(swerve, intake, otbIntake, handoff, shooter);
    }

    public Swerve getSwerve(){
        return swerve;
    }

    public Intake getIntake(){
        return intake;
    }

    public OtbIntake getOtbIntake(){
        return otbIntake;
    }

    public Handoff getHandoff(){
        return handoff;
    }

    public Shooter getShooter(){
        return shooter;
    }


}
