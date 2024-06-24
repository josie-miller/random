package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIO;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();


    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
    }

    public void periodic(){
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public Command setVelocity(double velocity, double ratio) {
        return this.run(() -> shooterIO.setVelocity(velocity, ratio));
    }

    public Command setVelocity(double velocity, double ratio, double time) {
        return this.run(() -> shooterIO.setVelocity(velocity, ratio))
        .withTimeout(time)
        .andThen(this.runOnce(() -> shooterIO.setVoltage(0)));
    }
    public Command setVoltage(double voltage) {
        return this.run(() -> shooterIO.setVoltage(voltage));
    }
}
