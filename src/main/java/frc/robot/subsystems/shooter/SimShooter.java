package frc.robot.subsystems.shooter;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.SimEncoder;

public class SimShooter implements ShooterIO {

    public static SimEncoder shooterEncoderSim;
    private DCMotorSim shooterMotorSim;

    public SimShooter() {
        shooterEncoderSim = new SimEncoder("shooter");
        DCMotor gearbox = DCMotor.getNeo550(1);
        LinearSystem<N2, N1, N2> plant = LinearSystemId.createDCMotorSystem(gearbox, 1, 1);
        shooterMotorSim = new DCMotorSim(plant, gearbox);
        SmartDashboard.putNumber("shooter current sim", 0);
        SmartDashboard.putNumber("shooter sim velocity", 0);
    }

    public void setMotor(double speed) {
        shooterMotorSim.setInput(speed);
    }

    public double getCurrent() {
        return SmartDashboard.getNumber("shooter current sim", -100);
    }

    public double getEncoderSpeed() {
        return SmartDashboard.getNumber("shooter sim velocity", -100);
    }

    // public void setCurrentLimit(int current) {
    //     SmartDashboard.getNumber("shooter current sim", current);
    // }

    public void periodicUpdate() {
        SmartDashboard.putNumber("shooter/current (A)", getCurrent());
    }
}
