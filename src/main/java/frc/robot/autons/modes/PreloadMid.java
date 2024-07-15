package frc.robot.autons.modes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.handoff.Handoff;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.otb_intake.OtbIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.commands.sequences.ShootSequence;

import frc.robot.constants.commandConstants;

public class PreloadMid extends SequentialCommandGroup {

    public PreloadMid(Swerve swerve, Intake intake, OtbIntake otbIntake, Handoff handoff, Shooter shooter){
        addRequirements(swerve, intake, otbIntake, handoff, shooter);
        addCommands(
            new InstantCommand(() -> swerve.setGyroStartingPosition(0)),
            new ShootSequence(intake, handoff, shooter, commandConstants.midShootTime, commandConstants.midShootRatio, commandConstants.handoffIntakeVoltage, commandConstants.handoffShooterVoltage, commandConstants.shootMidVelocity, commandConstants.midShortTime)
        );
    }

}
