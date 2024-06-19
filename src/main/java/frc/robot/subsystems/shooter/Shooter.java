package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIO;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();


    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
    }

    public void Loop(){
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public void setVelocity(double velocity, double ratio) {
        shooterIO.setVelocity(velocity, ratio);
    }

    public void setVoltage(double voltage) {
        shooterIO.setVoltage(voltage);
    }

}
