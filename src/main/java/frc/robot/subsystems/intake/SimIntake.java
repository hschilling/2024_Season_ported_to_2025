package frc.robot.subsystems.intake;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.SimEncoder;

public class SimIntake implements IntakeIO {

    public static SimEncoder leftIntakeEncoderSim;
    public static SimEncoder rightIntakeEncoderSim;
    private DCMotorSim leftIntakeMotorSim;
    private DCMotorSim rightIntakeMotorSim;

    public SimIntake() {
        leftIntakeEncoderSim = new SimEncoder("left intake");
        rightIntakeEncoderSim = new SimEncoder("right intake");

        DCMotor gearbox = DCMotor.getNeo550(1);
        LinearSystem<N2, N1, N2> plant = LinearSystemId.createDCMotorSystem(gearbox, 1, 1);
        leftIntakeMotorSim = new DCMotorSim(plant, gearbox);
        rightIntakeMotorSim = new DCMotorSim(plant, gearbox);

        SmartDashboard.putNumber("intakecurrent sim", 0);
        SmartDashboard.putNumber("intake sim velocity", 0);

    }

    public void setMotor(double speed) {
        leftIntakeMotorSim.setInput(speed);
        // rightIntakeMotorSim.setInput(speed);
    }

    public double getLeftCurrent() {
        return leftIntakeMotorSim.getCurrentDrawAmps();
    }

    public double getRightCurrent() {
        return rightIntakeMotorSim.getCurrentDrawAmps();
    }

    public double getLeftEncoderSpeed() {
        return leftIntakeEncoderSim.getSpeed();
    }

    public double getRightEncoderSpeed() {
        return rightIntakeEncoderSim.getSpeed();
    }

    public double getLeftEncoderPosition() {
        return leftIntakeEncoderSim.getDistance();
    }

    public double getRightEncoderPosition() {
        return rightIntakeEncoderSim.getDistance();
    }

    public void setLeftCurrentLimit(int current) {
        return;
    }

    public void setRightCurrentLimit(int current) {
        return;
    }

    public void periodicUpdate() {
        SmartDashboard.putNumber("intake/current (A)", getLeftCurrent());
        SmartDashboard.putNumber("intake/current (A)", getRightCurrent());
    }

}
