package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.otb_intake.OtbIntake;
import frc.robot.Constants;

public class SetPivot extends InstantCommand {
    private final OtbIntake otbIntake;

    public SetPivot(OtbIntake otbIntake) {
        super();
        this.otbIntake = otbIntake;
        addRequirements(otbIntake);
    }

    @Override
    public void initialize() {
        otbIntake.requestSetpoint(Constants.commandConstants.restingDegrees);
    }

    @Override
    public void end(boolean interrupted) {
    }
}