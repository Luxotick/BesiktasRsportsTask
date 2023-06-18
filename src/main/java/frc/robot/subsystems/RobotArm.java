package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;


import edu.wpi.first.math.controller.ArmFeedforward;

import frc.robot.Constants;

public class RobotArm extends SubsystemBase {
    
    private final DCMotor m_armGearbox = DCMotor.getFalcon500(2);

    private final Encoder m_encoder = new Encoder(0, 1);
    private final TalonFX m_motor = new TalonFX(0);


    private double m_armSetpointDegrees = Constants.m_armSetpointDegrees;

    private final SingleJointedArmSim m_armSim =
    new SingleJointedArmSim(
        m_armGearbox,
        Constants.kArmReduction,
        SingleJointedArmSim.estimateMOI(Constants.kArmLength, Constants.kArmMass),
        Constants.kArmLength,
        Constants.kMinAngleRads,
        Constants.kMaxAngleRads,
        true,
        VecBuilder.fill(Constants.kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
    );

    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
    private final MechanismLigament2d m_armTower =
        m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
    private final MechanismLigament2d m_arm = m_armPivot.append(new MechanismLigament2d("Arm",30,Units.radiansToDegrees(m_armSim.getAngleRads()),6,new Color8Bit(Color.kYellow)));
    
    public RobotArm() {
        m_encoder.setDistancePerPulse(Constants.kArmEncoderDistPerPulse);
    
        // Put Mechanism 2d to SmartDashboard
        SmartDashboard.putData("Arm Sim", m_mech2d);
        SmartDashboard.putData(m_encoder);
        m_armTower.setColor(new Color8Bit(Color.kBlue));
    }

    public void simulationPeriodic() {

        var m_talonFXSim = m_motor.getSimState();

        m_talonFXSim.setSupplyVoltage(12);

        var motorVoltage = m_talonFXSim.getMotorVoltage();

        m_armSim.setInput(motorVoltage);

        m_armSim.update(0.020);

        m_talonFXSim.setRawRotorPosition(Units.radiansToDegrees(m_armSim.getAngleRads()));
        m_talonFXSim.setRotorVelocity(m_armSim.getVelocityRadPerSec());

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

        m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    }

    public void reachSetpoint() {

    }

    ArmFeedforward feedforward = new ArmFeedforward(0.44,3.06,0.18);


    public void idle() {

        PositionVoltage positionVoltage = new PositionVoltage(0, false, feedforward.calculate(Units.degreesToRadians(m_armSetpointDegrees), 0) , 0, false);        
        m_motor.setControl(positionVoltage.withPosition(m_armSetpointDegrees));
        
    }

    public void stop() {
        m_motor.set(0.0);
    }

    public void close() {
      m_motor.close();
      m_encoder.close();
      m_mech2d.close();
      m_armPivot.close();
      m_arm.close();
    }

}
