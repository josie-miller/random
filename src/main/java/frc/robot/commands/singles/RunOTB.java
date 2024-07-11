package frc.robot.commands.singles;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.otb_intake.OtbIntake;

import frc.robot.constants.commandConstants;

public class RunOTB extends Command {
    private final OtbIntake otbIntake;
    private final double voltage;
    private final double angleDegrees;


    public RunOTB(OtbIntake otbIntake, double voltage, double angleDegrees) {
        this.otbIntake = otbIntake;
        this.voltage = voltage;
        this.angleDegrees = angleDegrees;
        addRequirements(otbIntake);
    }

    @Override
    public void initialize() {
        otbIntake.requestIntake(angleDegrees,voltage);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        otbIntake.requestIntake(commandConstants.restingDegrees, 0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
