package org.sert2521.rebuilt2026.subsystems

import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import dev.doglog.DogLog
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.rebuilt2026.ElectronicIDs
import org.sert2521.rebuilt2026.IndexerConstants
import org.sert2521.rebuilt2026.IndexerConstants.PULSE_DELAY_TIME
import org.sert2521.rebuilt2026.IndexerConstants.PULSE_SHOOT_TIME
import org.sert2521.rebuilt2026.TelemetryConstants
import yams.motorcontrollers.SmartMotorControllerConfig
import yams.motorcontrollers.local.SparkWrapper
import yams.telemetry.MechanismTelemetry

object Indexer : SubsystemBase() {
    private val indexerMotor = SparkMax(ElectronicIDs.INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
    private val kickerMotor = SparkMax(ElectronicIDs.KICKER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)

    private val indexerMotorConfig = SmartMotorControllerConfig(this)
        .withGearing(IndexerConstants.indexerGearing)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Indexer Motor", TelemetryConstants.indexerTelemetry)
        .withStatorCurrentLimit(Amps.of(50.0))
        .withMotorInverted(true)
        .withOpenLoopRampRate(Seconds.of(0.25))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withControlMode(SmartMotorControllerConfig.ControlMode.OPEN_LOOP)
    private val kickerMotorConfig = SmartMotorControllerConfig(this)
        .withGearing(IndexerConstants.kickerGearing)
        .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
        .withTelemetry("Kicker Motor", TelemetryConstants.indexerTelemetry)
        .withStatorCurrentLimit(Amps.of(60.0))
        .withMotorInverted(true)
        .withOpenLoopRampRate(Seconds.of(0.25))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withControlMode(SmartMotorControllerConfig.ControlMode.OPEN_LOOP)

    private val indexerSMC = SparkWrapper(indexerMotor, DCMotor.getNEO(1), indexerMotorConfig)
    private val kickerSMC = SparkWrapper(kickerMotor, DCMotor.getNEO(1), kickerMotorConfig)

    private val beambreak = DigitalInput(ElectronicIDs.INDEXER_BEAM_BREAK_ID)
    private val fullDebouncer = Debouncer(0.1, Debouncer.DebounceType.kFalling)
    private var beambreakClear = true

    private fun getBeamBreakBlocked(): Boolean {
        return !beambreakClear
    }

    private val telemetry = MechanismTelemetry()


    init {
        defaultCommand = default()

        telemetry.setupTelemetry("Indexer", indexerSMC)
        telemetry.setupTelemetry("Indexer", kickerSMC)
    }

    override fun periodic() {
        indexerSMC.updateTelemetry()
        kickerSMC.updateTelemetry()

        beambreakClear = fullDebouncer.calculate(beambreak.get())

        DogLog.log("Beambreak", getBeamBreakBlocked())


    }

    override fun simulationPeriodic() {
        indexerSMC.simIterate()
        kickerSMC.simIterate()
    }

    private fun setIndexerMotor(dutyCycle: Double) {
        indexerSMC.dutyCycle = dutyCycle
    }

    private fun setKickerMotor(dutyCycle: Double) {
        kickerSMC.dutyCycle = dutyCycle
    }

    private fun default(): Command {
        return runOnce {
            setIndexerMotor(IndexerConstants.MAIN_DEFAULT)
            setKickerMotor(IndexerConstants.KICKER_DEFAULT)
        }.andThen(
            Commands.idle()
        )
    }

    fun manualIndex(): Command {
        return runOnce {
            setIndexerMotor(IndexerConstants.MAIN_INDEXING)
            setKickerMotor(IndexerConstants.KICKER_INDEXING)
        }.andThen(
            Commands.idle()
        )
    }

    fun index(): Command {
        return run {
            if (getBeamBreakBlocked()) {
                setIndexerMotor(IndexerConstants.MAIN_INDEXING)
                setKickerMotor(IndexerConstants.KICKER_INDEXING)
            } else {
                setIndexerMotor(IndexerConstants.MAIN_DEFAULT)
                setKickerMotor(IndexerConstants.KICKER_DEFAULT)
            }
        }
    }

    private fun shoot(): Command {
        return runOnce {
            setIndexerMotor(IndexerConstants.MAIN_KICKING)
            setKickerMotor(IndexerConstants.KICKER_KICKING)
        }.andThen(
            Commands.idle()
        )
    }

    fun pulse(): Command {
        return shoot().withTimeout(PULSE_SHOOT_TIME)
            .andThen(index().withTimeout(PULSE_DELAY_TIME))
            .repeatedly()
    }

    fun reverse(): Command {
        return runOnce {
            setIndexerMotor(IndexerConstants.MAIN_REVERSE)
            setKickerMotor(IndexerConstants.KICKER_REVERSE)
        }.andThen(
            Commands.idle()
        )
    }
}