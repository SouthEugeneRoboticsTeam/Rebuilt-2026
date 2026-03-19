package org.sert2521.rebuilt2026.subsystems.hooded_shooter

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Velocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sert2521.rebuilt2026.ElectronicIDs
import org.sert2521.rebuilt2026.ShooterConstants
import org.sert2521.rebuilt2026.TelemetryConstants
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import yams.telemetry.MechanismTelemetry
import java.util.function.Supplier

object Roller : SubsystemBase() {
    private val motorRoller = SparkMax(ElectronicIDs.SHOOTER_ROLLER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val motorConfigRoller = SmartMotorControllerConfig(this)
        .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
        .withMotorInverted(false)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withStatorCurrentLimit(Units.Amps.of(40.0))
        .withFeedforward(SimpleMotorFeedforward(ShooterConstants.R_S, ShooterConstants.R_V))
        .withClosedLoopController(ShooterConstants.R_P, 0.0, ShooterConstants.R_D)
        .withGearing(ShooterConstants.rollerGearing)
        .withTelemetry("Shooter Roller Motor", TelemetryConstants.HOODED_SHOOTER_TELEMETRY)

    private val smc = SparkWrapper(motorRoller, DCMotor.getNEO(1), motorConfigRoller)
    private val telemetry = MechanismTelemetry()

    init {
        telemetry.setupTelemetry("Shooter Roller", smc)
    }

    override fun periodic() {
        smc.updateTelemetry()
    }

    override fun simulationPeriodic() {
        smc.simIterate()
    }

    fun setVelocity(velocity:Supplier<AngularVelocity>): Command {
        return run {
            smc.setVelocity(velocity.get())
        }
    }

    fun setVoltage(voltage:Supplier<Voltage>): Command {
        return run {
            smc.voltage = voltage.get()
        }
    }

    fun stop():Command {
        return runOnce {
            smc.dutyCycle = 0.0
        }.andThen(Commands.idle())
    }
}