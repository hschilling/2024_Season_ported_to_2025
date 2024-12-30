package frc.robot.subsystems.Indexer;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.SimEncoder;

public class SimIndexer implements IndexerIO {
    public static SimEncoder indexerEncoderSim;
    private DCMotorSim indexerMotorSim;
    public boolean isIntooked = false;

    public SimIndexer() {
        indexerEncoderSim = new SimEncoder("indexer");
        DCMotor gearbox = DCMotor.getNeo550(1);
        LinearSystem<N2, N1, N2> plant = LinearSystemId.createDCMotorSystem(gearbox, 1, 1);
        indexerMotorSim = new DCMotorSim(plant, gearbox);

        SmartDashboard.putNumber("indexer current sim", 0);
        SmartDashboard.putNumber("indexer sim velocity", 0);
    }

    public void setMotor(double speed) {
        indexerMotorSim.setInput(speed);
    }

    public double getCurrent() {
        return indexerMotorSim.getCurrentDrawAmps();
    }

    public double getEncoderSpeed() {
        return indexerEncoderSim.getSpeed();
    }

    public double getEncoderPosition() {
        return indexerEncoderSim.getDistance();
    }

    // public void setCurrentLimit(int current) {
    //     return;
    // }

    public void periodicUpdate() {
        SmartDashboard.putNumber("Driver/indexer/current (A)", getCurrent());
    }

    public boolean getIsBeamBroken() {
        return false;
    }

    @Override
    public void setIsOverride() {
        
    }
}
