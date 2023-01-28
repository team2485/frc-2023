package frc.robot.subsystems.drive;

import frc.robot.Constants.AutoConstants.TelescopeConstants;
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
import frc.WarlordsLib.motorcontrol.WL_SparkMax;
import frc.WarlordsLib.motorcontrol.WL_TalonFX;
import frc.robot.Constants;
//import frc.team2485.WarlordsLib.motorcontrol.WL_TalonFX;
//import frc.team2485.WarlordsLib.sendableRichness.SR_SimpleMotorFeedforward;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Telescope {
    private final WL_SparkMax m_spark = new WL_SparkMax(frc.robot.Constants.AutoConstants.TelescopeConstants.kTelescopeSparkPort);    //extremely sus code, review later
    private final WPI_TalonFX m_talon = new WL_TalonFX(frc.robot.Constants.AutoConstants.TelescopeConstants.kTelescopePort);          //sus :/


     private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(
        frc.robot.Constants.AutoConstants.TelescopeConstants.kSTelescopeVolts, TelescopeConstants.kVTelescopeVoltSecondsPerMeter, frc.robot.Constants.kATelescopeVolt);



    public Telescope(){

        m_spark.enableVoltageCompensation(Constants.kNominalVoltage);
        m_spark.setSmartCurrentLimit(TelescopeConstants.kTelescopeSmartCurrentLimitAmps);
        m_spark.setSecondaryCurrentLimit(TelescopeConstants.kTelescopeImmediateCurrentLimitAmps);


        TalonFXConfiguration telescopeTalonFXConfiguration = new TalonFXConfiguration();
        telescopeTalonFXConfiguration.voltageCompSaturation = Constants.kNominalVoltage;
        telescopeTalonFXConfiguration.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_1Ms;
        telescopeTalonFXConfiguration.velocityMeasurementWindow = 1; 

        


    }

    public void setPWM(double pwm) {
        m_talon.set(pwm);
    }

}
