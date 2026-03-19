package org.sert2521.rebuilt2026.subsystems.hooded_shooter

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.measure.AngularVelocity
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

object Flywheel : SubsystemBase() {
    private val motorLeft = SparkMax(ElectronicIDs.FLYWHEEL_LEFT_ID, SparkLowLevel.MotorType.kBrushless)
    private val motorRight = SparkMax(ElectronicIDs.FLYWHEEL_RIGHT_ID, SparkLowLevel.MotorType.kBrushless)

    private val flywheelMotorConfig = {
        SmartMotorControllerConfig(this)
            .withClosedLoopController(ShooterConstants.F_P, 0.0, ShooterConstants.F_D)
            .withFeedforward(
                SimpleMotorFeedforward(
                    ShooterConstants.F_S,
                    ShooterConstants.F_V,
                    ShooterConstants.F_A
                )
            )
            .withGearing(ShooterConstants.shooterGearing)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.COAST)
            .withStatorCurrentLimit(Units.Amps.of(40.0))
    }

    private val flywheelMotorConfigLeft = flywheelMotorConfig()
        .withTelemetry("Flywheel Motor Left", TelemetryConstants.HOODED_SHOOTER_TELEMETRY)
        .withMotorInverted(false)
    private val flywheelMotorConfigRight = flywheelMotorConfig()
        .withTelemetry("Flywheel Motor Right", TelemetryConstants.HOODED_SHOOTER_TELEMETRY)
        .withMotorInverted(true)

    private val leftSMC = SparkWrapper(motorLeft, DCMotor.getNEO(1), flywheelMotorConfigLeft)
    private val rightSMC = SparkWrapper(motorRight, DCMotor.getNEO(1), flywheelMotorConfigRight)
    private val telemetry = MechanismTelemetry()
    private var revved = false

    init {
        telemetry.setupTelemetry("Flywheels", leftSMC)
        telemetry.setupTelemetry("Flywheels", rightSMC)
    }

    override fun periodic() {
        leftSMC.updateTelemetry()
        rightSMC.updateTelemetry()
    }

    override fun simulationPeriodic() {
        leftSMC.simIterate()
        rightSMC.simIterate()
    }

    fun setVelocity(velocity:Supplier<AngularVelocity>):Command{
        return run {
            leftSMC.setVelocity(velocity.get())
            rightSMC.setVelocity(velocity.get())
            revved = MathUtil.isNear(velocity.get().`in`(RPM), leftSMC.mechanismVelocity.`in`(RPM), 20.0)
        }
    }

    fun isRevved():Boolean {
        return revved
    }

    fun stop():Command{
        return runOnce{
            leftSMC.dutyCycle = 0.0
            rightSMC.dutyCycle = 0.0
        }.andThen(Commands.idle())
    }
}