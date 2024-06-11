package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.otb_intake.otbIntake;

public class SetPivot2 extends InstantCommand {
    private final otbIntake otbIntake;
    private double degrees;    

    public SetPivot2(otbIntake otbIntake, double degrees) {
        super();
        this.otbIntake = otbIntake;
        this.degrees = degrees;
        addRequirements(otbIntake);
    }

    @Override
    public void initialize() {
        otbIntake.requestSetpoint(degrees);
    }

    @Override
    public void end(boolean interrupted) {
    }
}