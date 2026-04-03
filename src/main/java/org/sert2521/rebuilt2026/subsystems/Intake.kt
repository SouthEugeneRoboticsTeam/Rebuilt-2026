package org.sert2521.rebuilt2026.subsystems

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.rebuilt2026.ElectronicIDs
import org.sert2521.rebuilt2026.GrintakeConstants
import org.sert2521.rebuilt2026.TelemetryConstants
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import yams.telemetry.MechanismTelemetry

object Intake : SubsystemBase() {
    private val rollerMotor = SparkMax(ElectronicIDs.GRINTAKE_ROLLER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val rollerMotorConfig = SmartMotorControllerConfig(this)
        .withGearing(GrintakeConstants.rollerGearing)
        .withMotorInverted(false)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Roller Motor", TelemetryConstants.GRINTAKE_TELEMETRY)
        .withControlMode(SmartMotorControllerConfig.ControlMode.OPEN_LOOP)
        .withStatorCurrentLimit(Amps.of(40.0))

    private val rollerSMC = SparkWrapper(
        rollerMotor, DCMotor.getNEO(1),
        rollerMotorConfig
    )
    private val telemetry = MechanismTelemetry()

    init {
        defaultCommand = stop()

        telemetry.setupTelemetry("Grintake", rollerSMC)
    }

    override fun periodic() {
        rollerSMC.updateTelemetry()
    }

    private fun setRollerMotor(voltage: Voltage) {
        rollerSMC.voltage = voltage
    }

    fun intake(): Command {
        return runOnce {
            setRollerMotor(GrintakeConstants.intakeVoltage)
        }.andThen(Commands.idle())
    }

    fun fullSpeedIntake(): Command {
        return runOnce {
            setRollerMotor(Volts.of(12.0))
        }.andThen(Commands.idle())
    }

    fun reverse(): Command {
        return runOnce {
            setRollerMotor(GrintakeConstants.reverseVoltage)
        }.andThen(
            Commands.idle()
        )
    }

    fun stop(): Command {
        return runOnce {
            setRollerMotor(Volts.zero())
        }.andThen(
            Commands.idle()
        )
    }
}