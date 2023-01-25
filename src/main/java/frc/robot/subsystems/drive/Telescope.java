package frc.robot.subsystems.drive;

import static frc.robot.Constants.TelescopeConstants;
//import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WarlordsLib.motorcontrol.WL_TalonFX;
import frc.robot.Constants;
//import frc.team2485.WarlordsLib.motorcontrol.WL_TalonFX;
//import frc.team2485.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Telescope {
    private final WPI_TalonFX m_talon = new WL_TalonFX(frc.robot.Constants.kTelescopePort);


     private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(
          frc.robot.Constants.kSTelescopeVolts, frc.robot.Constants.kGTelescopeVolts, frc.robot.Constants.kVTelescopeVoltSecondsPerMeter, frc.robot.Constants.TelescopeConstantskATelescopeVoltSecondsSquaredPerMeter);



    public Telescope(){
        TalonFXConfiguration telescopeTalonFXConfiguration = new TalonFXConfiguration();
        telescopeTalonConfig.voltageCompSaturation = Constants.kNominalVoltage;
        telescopeTalonConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_1Ms;
        telescopeTalonConfig.velocityMeasurementWindow = 1; 

        telescope 
        


    }

}
