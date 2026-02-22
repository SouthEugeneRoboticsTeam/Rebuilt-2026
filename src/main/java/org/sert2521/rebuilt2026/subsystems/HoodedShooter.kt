package org.sert2521.rebuilt2026.subsystems

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.AngularVelocityUnit
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.rebuilt2026.ElectronicIDs
import org.sert2521.rebuilt2026.HoodedShooterConstants
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import yams.telemetry.MechanismTelemetry
import javax.xml.crypto.dsig.spec.RSAPSSParameterSpec
import kotlin.math.pow

object HoodedShooter : SubsystemBase() {
    private val motorLeft = SparkMax(ElectronicIDs.FLYWHEEL_LEFT_ID, SparkLowLevel.MotorType.kBrushless)
    private val motorRight = SparkMax(ElectronicIDs.FLYWHEEL_RIGHT_ID, SparkLowLevel.MotorType.kBrushless)
    private val motorRoller = SparkMax(ElectronicIDs.SHOOTER_ROLLER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val feedforward = SimpleMotorFeedforward(HoodedShooterConstants.S, HoodedShooterConstants.V,
        HoodedShooterConstants.A)
    private val pidLeft = PIDController(0.0, 0.0, HoodedShooterConstants.D)
    private val pidRight = PIDController(0.0, 0.0, HoodedShooterConstants.D)

    private val flywheelMotorConfig = {
        SmartMotorControllerConfig(this)
            .withClosedLoopController(HoodedShooterConstants.P, 0.0, HoodedShooterConstants.D)
            .withFeedforward(
                SimpleMotorFeedforward(
                    HoodedShooterConstants.S,
                    HoodedShooterConstants.V,
                    HoodedShooterConstants.A
                )
            )
            .withGearing(HoodedShooterConstants.shooterGearing)
            .withIdleMode(SmartMotorControllerConfig.MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(40.0))
    }

    private val flywheelMotorConfigLeft = flywheelMotorConfig()
        .withTelemetry("Flywheel Motor Left", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withMotorInverted(false)
    private val flywheelMotorConfigRight = flywheelMotorConfig()
        .withTelemetry("Flywheel Motor Right", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withMotorInverted(true)

    private val motorConfigRoller = SmartMotorControllerConfig(this)
        .withControlMode(SmartMotorControllerConfig.ControlMode.OPEN_LOOP)
        .withMotorInverted(false)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40.0))
        .withGearing(HoodedShooterConstants.rollerGearing)
        .withTelemetry("Hood Rollers Motor", SmartMotorControllerConfig.TelemetryVerbosity.LOW)

    private val flywheelLeftSMC = SparkWrapper(motorLeft, DCMotor.getNEO(1), flywheelMotorConfigLeft)
    private val flywheelRightSMC = SparkWrapper(motorRight, DCMotor.getNEO(1), flywheelMotorConfigRight)
    private val rollerSMC = SparkWrapper(motorRoller, DCMotor.getNEO(1), motorConfigRoller)
    private val telemetry = MechanismTelemetry()

    private var shooterSetpoint = RPM.zero()

    init {
        defaultCommand = run {}

        telemetry.setupTelemetry("HoodedShooter", flywheelLeftSMC)
        telemetry.setupTelemetry("HoodedShooter", flywheelRightSMC)
        telemetry.setupTelemetry("HoodedShooter", rollerSMC)
    }

    override fun periodic() {
        flywheelLeftSMC.updateTelemetry()
        flywheelRightSMC.updateTelemetry()
        rollerSMC.updateTelemetry()
    }

    override fun simulationPeriodic() {
        flywheelLeftSMC.simIterate()
        flywheelRightSMC.simIterate()
        rollerSMC.simIterate()
    }

    private fun runPID(): Command{
        return run {
            flywheelLeftSMC.voltage = Volts.of(
                pidLeft.calculate(flywheelLeftSMC.mechanismVelocity.`in`(RotationsPerSecond), shooterSetpoint.`in`(RotationsPerSecond))
                + feedforward.calculate(shooterSetpoint.`in`(RotationsPerSecond))
                + HoodedShooterConstants.Q * (shooterSetpoint.`in`(RotationsPerSecond) - flywheelLeftSMC.mechanismVelocity.`in`(RotationsPerSecond)).pow(2))
            flywheelRightSMC.voltage = Volts.of(
                pidRight.calculate(flywheelRightSMC.mechanismVelocity.`in`(RotationsPerSecond), shooterSetpoint.`in`(RotationsPerSecond))
                        + feedforward.calculate(shooterSetpoint.`in`(RotationsPerSecond))
                        + HoodedShooterConstants.Q * (shooterSetpoint.`in`(RotationsPerSecond) - flywheelRightSMC.mechanismVelocity.`in`(RotationsPerSecond)).pow(2))
        }
    }

    private fun setVelocitiesCommand(flywheelsVelocity: AngularVelocity, rollerDutyCycle: Double): Command {
        return runOnce {
            flywheelLeftSMC.setVelocity(flywheelsVelocity)
            flywheelRightSMC.setVelocity(flywheelsVelocity)
            rollerSMC.dutyCycle = rollerDutyCycle
            shooterSetpoint = flywheelsVelocity
        }
    }

    fun shoot():Command {
        return setVelocitiesCommand(HoodedShooterConstants.shootTarget, HoodedShooterConstants.shootRollerDutyCycle)
    }

    fun rev():Command {
        return setVelocitiesCommand(HoodedShooterConstants.shootTarget, 0.0)
    }

    fun stop(): Command {
        return setVelocitiesCommand(RPM.of(0.0), 0.0)
    }

    fun isRevved():Boolean {
        return MathUtil.isNear((flywheelLeftSMC.mechanismVelocity / 2.0 + flywheelRightSMC.mechanismVelocity / 2.0).`in`(RPM),
            shooterSetpoint.`in`(RPM), 100.0)
    }
}