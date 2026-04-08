package org.sert2521.rebuilt2026.subsystems.hooded_shooter

import com.ctre.phoenix6.configs.MagnetSensorConfigs
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import dev.doglog.DogLog
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.rebuilt2026.ElectronicIDs
import org.sert2521.rebuilt2026.ShooterConstants
import org.sert2521.rebuilt2026.TelemetryConstants
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import yams.telemetry.MechanismTelemetry
import java.util.function.Supplier

object Hood : SubsystemBase() {
    private val hoodMotor = SparkMax(ElectronicIDs.HOOD_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
    private val absoluteEncoder = CANcoder(ElectronicIDs.HOOD_ABSOLUTE_ENCODER_ID)
    private val absolutePosition = absoluteEncoder.position.asSupplier()

    private val hoodConfig = SmartMotorControllerConfig(this)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withGearing(ShooterConstants.hoodGearing)
        .withMotorInverted(false)
        .withStatorCurrentLimit(Amps.of(30.0))
        .withClosedLoopController(ShooterConstants.H_P, 0.0, ShooterConstants.H_D)
        .withFeedforward(SimpleMotorFeedforward(ShooterConstants.H_S, 0.0))
        .withTelemetry("Hood Motor", TelemetryConstants.hoodedShooterTelemetry)

    private val debouncer = Debouncer(0.4, Debouncer.DebounceType.kRising)

    private val smc = SparkWrapper(hoodMotor, DCMotor.getNEO(1), hoodConfig)
    private val telemetry = MechanismTelemetry()

    init {
        telemetry.setupTelemetry("Hood", smc)
        absoluteEncoder.configurator.apply(
            MagnetSensorConfigs()
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        )
    }

    // 0.382
    override fun periodic() {
        smc.setEncoderPosition((absolutePosition.get() + ShooterConstants.hoodOffset) / ShooterConstants.HOOD_ABSOLUTE_ENCODER_GEARING)
        DogLog.log("Absolute Encoder", absolutePosition.get())
        DogLog.log(
            "Absolute Encoder with gearing",
            absolutePosition.get() / ShooterConstants.HOOD_ABSOLUTE_ENCODER_GEARING
        )
        smc.updateTelemetry()
    }

    override fun simulationPeriodic() {
        smc.simIterate()
    }

    fun setPosition(position: Supplier<Angle>): Command {
        return run {
            smc.setPosition(position.get())
        }
    }

    fun stop(): Command {
        return runOnce {
            smc.dutyCycle = 0.0
        }.andThen(Commands.idle())
    }

    fun reZero(): Command {
        return runOnce {
            smc.dutyCycle = -0.2
        }.andThen(
            Commands.idle()
        ).until {
            debouncer.calculate(smc.statorCurrent > Amps.of(20.0))
        }.andThen(
            runOnce {
                absoluteEncoder.setPosition(Rotations.zero())
            }
        )
    }
}