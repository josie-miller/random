package frc.robot.subsystems.otb_intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class OtbIntake extends SubsystemBase {
    private final OtbIntakeIO otbintakeIO;
    private OtbIntakeIOInputsAutoLogged inputs = new OtbIntakeIOInputsAutoLogged();

    private double setpointVolts;

    public OtbIntake(OtbIntakeIO otbintakeIO) {
        this.otbintakeIO = otbintakeIO;
        setpointVolts = 0.0;
    }

    

    @Override    
    public void periodic(){
        otbintakeIO.updateInputs(inputs);
        Logger.processInputs("OTB_Intake", inputs);
    }

    public void requestIntakeVoltage(double voltage) {
        setpointVolts = voltage;
        otbintakeIO.setIntakeVoltage(setpointVolts);
    }

    public double getStatorCurrent(){
        return inputs.intakeCurrent;
    }
}