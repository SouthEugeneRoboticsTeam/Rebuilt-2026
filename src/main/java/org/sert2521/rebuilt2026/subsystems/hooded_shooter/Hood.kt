package org.sert2521.rebuilt2026.subsystems.hooded_shooter

import com.ctre.phoenix6.hardware.CANcoder
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import dev.doglog.DogLog
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sert2521.rebuilt2026.ElectronicIDs
import org.sert2521.rebuilt2026.ShooterConstants
import org.sert2521.rebuilt2026.TelemetryConstants
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import yams.telemetry.MechanismTelemetry
import java.time.temporal.TemporalQueries.offset
import java.util.function.Supplier

object Hood : SubsystemBase() {
    private val hoodMotor = SparkMax(ElectronicIDs.HOOD_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
    private val absoluteEncoder = CANcoder(45)
    private val absolutePosition = absoluteEncoder.position.asSupplier()

    private val hoodConfig = SmartMotorControllerConfig(this)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withGearing(ShooterConstants.hoodGearing)
        .withMotorInverted(false)
        .withStatorCurrentLimit(Amps.of(30.0))
        .withClosedLoopController(ShooterConstants.H_P, 0.0, ShooterConstants.H_D)
        .withTelemetry("Hood Motor", TelemetryConstants.HOODED_SHOOTER_TELEMETRY)

    private val smc = SparkWrapper(hoodMotor, DCMotor.getNEO(1), hoodConfig)
    private val telemetry = MechanismTelemetry()

    init {
        telemetry.setupTelemetry("Hood", smc)
    }

    override fun periodic() {
        // smc.setEncoderPosition(absolutePosition.get()+ShooterConstants.hoodOffset)
        DogLog.log("Absolute Encoder", absolutePosition.get())
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

    fun stop():Command {
        return runOnce {
            smc.dutyCycle = 0.0
        }.andThen(Commands.idle())
    }
}