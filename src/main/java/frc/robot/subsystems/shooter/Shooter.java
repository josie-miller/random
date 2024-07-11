package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Conversions;

public class Shooter extends SubsystemBase{
    
    private final ShooterIO shooterIO;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private double setpointVelocity;


    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
        this.setpointVelocity = setpointVelocity;
    }

    public void periodic(){
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public void setVelocity(double velocity, double ratio) {
        setpointVelocity = velocity;
        shooterIO.setVelocity(setpointVelocity, ratio);
    }

    public void zeroVelocity() {
        setpointVelocity = 0;
        shooterIO.zeroVelocity();
    }

}

