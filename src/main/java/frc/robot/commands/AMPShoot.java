package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.handoff.Handoff;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AMPShoot extends Command {
    public AMPShoot(Intake intake, Handoff handoff, Shooter shooter) {
        new ShootSequence(intake, handoff, shooter, Constants.commandConstants.AMPShootTime, Constants.commandConstants.AMPShootRatio, Constants.commandConstants.handoffIntakeVoltage, Constants.commandConstants.handoffShooterVoltage, Constants.commandConstants.AMPShooterVelocity, Constants.commandConstants.AMPShortTime);
    }
}