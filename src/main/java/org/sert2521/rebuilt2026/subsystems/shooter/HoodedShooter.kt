package org.sert2521.rebuilt2026.subsystems.shooter

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import dev.doglog.DogLog
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.rebuilt2026.ElectronicIDs
import org.sert2521.rebuilt2026.HoodedShooterConstants
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import yams.telemetry.MechanismTelemetry
import kotlin.math.pow
import kotlin.math.sign

object HoodedShooter : SubsystemBase() {
    private val motorLeft = SparkMax(ElectronicIDs.FLYWHEEL_LEFT_ID, SparkLowLevel.MotorType.kBrushless)
    private val motorRight = SparkMax(ElectronicIDs.FLYWHEEL_RIGHT_ID, SparkLowLevel.MotorType.kBrushless)
    private val motorRoller = SparkMax(ElectronicIDs.SHOOTER_ROLLER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val feedforward = SimpleMotorFeedforward(
        HoodedShooterConstants.S, HoodedShooterConstants.V,
        HoodedShooterConstants.A
    )
    private val pidLeft = PIDController(0.0, 0.0, HoodedShooterConstants.D)
    private val pidRight = PIDController(0.0, 0.0, HoodedShooterConstants.D)
    private val debouncer = Debouncer(0.2, Debouncer.DebounceType.kFalling)

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
            .withStatorCurrentLimit(Units.Amps.of(40.0))
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
        .withStatorCurrentLimit(Units.Amps.of(40.0))
        .withGearing(HoodedShooterConstants.rollerGearing)
        .withTelemetry("Hood Rollers Motor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)

    private val flywheelLeftSMC = SparkWrapper(motorLeft, DCMotor.getNEO(1), flywheelMotorConfigLeft)
    private val flywheelRightSMC = SparkWrapper(motorRight, DCMotor.getNEO(1), flywheelMotorConfigRight)
    private val rollerSMC = SparkWrapper(motorRoller, DCMotor.getNEO(1), motorConfigRoller)
    private val telemetry = MechanismTelemetry()

    private var shooterSetpoint = Units.RotationsPerSecond.zero()
    private var rollerSetpoint = Units.Volts.of(0.0)
    private var revved = false

    init {
        defaultCommand = run { rollerSMC.voltage = rollerSetpoint}

        telemetry.setupTelemetry("HoodedShooter", flywheelLeftSMC)
        telemetry.setupTelemetry("HoodedShooter", flywheelRightSMC)
        telemetry.setupTelemetry("HoodedShooter", rollerSMC)
    }

    override fun periodic() {
        flywheelLeftSMC.updateTelemetry()
        flywheelRightSMC.updateTelemetry()
        rollerSMC.updateTelemetry()

        revved = debouncer.calculate(flywheelLeftSMC.mechanismVelocity > shooterSetpoint)
    }

    override fun simulationPeriodic() {
        flywheelLeftSMC.simIterate()
        flywheelRightSMC.simIterate()
        rollerSMC.simIterate()
    }

    private fun runPID(): Command {
        return run {
            DogLog.log("Q output", (HoodedShooterConstants.Q * (shooterSetpoint.`in`(Units.RotationsPerSecond) - flywheelLeftSMC.mechanismVelocity.`in`(
                Units.RotationsPerSecond
            )).pow(2)
                    * sign(shooterSetpoint.`in`(Units.RotationsPerSecond) - flywheelLeftSMC.mechanismVelocity.`in`(Units.RotationsPerSecond))))

            flywheelLeftSMC.voltage = Units.Volts.of(
                pidLeft.calculate(flywheelLeftSMC.mechanismVelocity.`in`(Units.RotationsPerSecond), shooterSetpoint.`in`(
                    Units.RotationsPerSecond
                ))
                + feedforward.calculate(shooterSetpoint.`in`(Units.RotationsPerSecond))
                + (HoodedShooterConstants.Q * (shooterSetpoint.`in`(Units.RotationsPerSecond) - flywheelLeftSMC.mechanismVelocity.`in`(
                    Units.RotationsPerSecond
                )).pow(2)
                * sign(shooterSetpoint.`in`(Units.RotationsPerSecond) - flywheelLeftSMC.mechanismVelocity.`in`(Units.RotationsPerSecond)))
            )
            flywheelRightSMC.voltage = Units.Volts.of(
                pidRight.calculate(flywheelRightSMC.mechanismVelocity.`in`(Units.RotationsPerSecond), shooterSetpoint.`in`(
                    Units.RotationsPerSecond
                ))
                        + feedforward.calculate(shooterSetpoint.`in`(Units.RotationsPerSecond))
                        + (HoodedShooterConstants.Q * (shooterSetpoint.`in`(Units.RotationsPerSecond) - flywheelRightSMC.mechanismVelocity.`in`(
                    Units.RotationsPerSecond
                )).pow(2)
                        * sign(
                    shooterSetpoint.`in`(Units.RotationsPerSecond) - flywheelRightSMC.mechanismVelocity.`in`(
                        Units.RotationsPerSecond
                    )
                ))
            )
        }
    }

    private fun setSetpoints(flywheelVelocity: AngularVelocity, rollerVoltage: Voltage){
        shooterSetpoint = flywheelVelocity
        rollerSetpoint = rollerVoltage
    }

    private fun setVelocitiesCommand(flywheelsVelocityPrimary: AngularVelocity, rollerVoltage: Voltage): Command {
        return runOnce {
            setSetpoints(flywheelsVelocityPrimary, rollerVoltage)
            DogLog.log("Target Velocity", flywheelsVelocityPrimary.`in`(Units.RPM))
            flywheelLeftSMC.setVelocity(flywheelsVelocityPrimary)
            flywheelRightSMC.setVelocity(flywheelsVelocityPrimary)
            rollerSMC.voltage = rollerVoltage
        }.andThen(
            run {
                rollerSMC.voltage = rollerVoltage
            }
        )
    }

    private fun shootRoutine(primaryVelocity: AngularVelocity, secondaryVelocity: AngularVelocity, rollerVoltage: Voltage): Command {
        return setVelocitiesCommand(primaryVelocity, rollerVoltage)
            .until { !isRevved() }
            .andThen(setVelocitiesCommand(secondaryVelocity, rollerVoltage))
    }

    fun shoot(): Command {
        return shootRoutine(
            HoodedShooterConstants.primaryShootFlywheel,
            HoodedShooterConstants.secondaryShootFlywheel,
            HoodedShooterConstants.shootRollerDutyCycle
        )
    }

    fun rev(): Command {
        return setVelocitiesCommand(HoodedShooterConstants.primaryShootFlywheel, HoodedShooterConstants.shootRollerDutyCycle).asProxy()
    }

    fun stop(): Command {
        return setVelocitiesCommand(Units.RotationsPerSecond.of(0.0), Units.Volts.of(0.0)).asProxy()
    }

    fun isRevved():Boolean {
        return revved
    }
}