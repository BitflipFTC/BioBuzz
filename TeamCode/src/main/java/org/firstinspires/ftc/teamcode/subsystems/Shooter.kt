package org.firstinspires.ftc.teamcode.subsystems

import club.bitflip.utils.hardware.MotorEx
import club.bitflip.utils.hardware.ServoEx
import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.InterpolatedLookupTable
import kotlin.math.PI
import kotlin.math.cos

@Configurable
class Shooter(hardwareMap: HardwareMap, telemetry: Telemetry) : Subsystem(hardwareMap, telemetry) {

    companion object {
        @JvmField
        var gain = 0.01

        // Flywheel geometry. The empirical distance/RPM and distance/time-in-air
        // tables were measured with this flywheel, so they already include its
        // diameter, the 14-degree hood, and the ball's slip against the wheel.
        const val FLYWHEEL_DIAMETER_MM = 72.0
        const val FLYWHEEL_DIAMETER_INCHES = FLYWHEEL_DIAMETER_MM / 25.4
        const val HOOD_ANGLE_DEGREES = 14.0

        fun surfaceSpeedFromRpm(rpm: Double): Double {
            return rpm * PI * FLYWHEEL_DIAMETER_INCHES / 60.0
        }

        fun rpmFromSurfaceSpeed(surfaceSpeed: Double): Double {
            return surfaceSpeed * 60.0 / (PI * FLYWHEEL_DIAMETER_INCHES)
        }

        val distanceArray = doubleArrayOf(
            32.0,
            48.0,
            64.0,
            80.0,
            96.0
        )
        val speedArray = doubleArrayOf(
            2875.0,
            3000.0,
            3125.0,
            3250.0,
            3375.0
        )
        val timeInAirArray = doubleArrayOf(
            0.5,
            0.5,
            0.6,
            0.65,
            0.7
        )

        val velocityLookupTable = InterpolatedLookupTable(
            distanceArray,
            speedArray
        )

        val timeInAirLookupTable = InterpolatedLookupTable(
            distanceArray,
            timeInAirArray
        )
    }

    var timeInAir = 0.5

    private val motor = MotorEx("shooter").reverse().float()
    private var error = 0.0
    private var lastError = 0.0

    private var output = 0.0
    private var tbh = 0.0
    var bangbang = true

    var targetFlywheelRPM = 0.0
    var flywheelRPM = 0.0
        private set

    var targetDistance = 32.0
        private set

    // Horizontal ball speed implied by the empirical distance/time-in-air table.
    var tableHorizontalSpeed = 0.0
        private set

    // Theoretical surface speed of the flywheel at its measured RPM.
    val flywheelSurfaceSpeed: Double
        get() = surfaceSpeedFromRpm(flywheelRPM)

    // Horizontal ball speed as a fraction of the flywheel's surface speed.
    // This includes both the 14-degree hood projection and wheel slip.
    val effectiveTransferRatio: Double
        get() = if (flywheelSurfaceSpeed > 1e-6) {
            tableHorizontalSpeed / flywheelSurfaceSpeed
        } else {
            0.0
        }

    // Slip ratio after removing the 14-degree hood projection.
    // Values below 1 mean the ball leaves slower than the wheel surface.
    val effectiveSlipRatio: Double
        get() {
            val projectedSurfaceSpeed = flywheelSurfaceSpeed *
                    cos(Math.toRadians(HOOD_ANGLE_DEGREES))

            return if (projectedSurfaceSpeed > 1e-6) {
                tableHorizontalSpeed / projectedSurfaceSpeed
            } else {
                0.0
            }
        }

    fun controlLoop() {
        flywheelRPM = (motor.velocity / 28) * 60

        lastError = error
        error = targetFlywheelRPM - flywheelRPM

        if (error >= 60.0 && bangbang) {
            motor.power = 1.0
            output = targetFlywheelRPM * (1.0 / 5000.0)
            tbh = output
        } else {
            output += gain * error
            output = output.coerceIn(-0.05, 1.0)

            if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
                output = 0.5 * (output + tbh)
                tbh = output
            }

            output = output.coerceIn(0.00, 1.0)

            motor.power = output
        }
    }

    fun setTargetState (distance: Double) {
        targetDistance = distance
        timeInAir = timeInAirLookupTable.calculate(distance)

        targetFlywheelRPM = velocityLookupTable.calculate(distance)

        tableHorizontalSpeed = if (timeInAir > 1e-6) {
            distance / timeInAir
        } else {
            0.0
        }
    }

    override fun loop() {
        controlLoop()

        telemetry.run {
            addData("Target flywheel RPM", targetFlywheelRPM)
            addData("Actual flywheel RPM", flywheelRPM)
            addData("Motor power", output)

            addData("Target distance (in)", targetDistance)
            addData("Flywheel diameter (mm)", FLYWHEEL_DIAMETER_MM)
            addData("Flywheel surface speed (in/s)", flywheelSurfaceSpeed)
            addData("Table horizontal speed (in/s)", tableHorizontalSpeed)
            addData("Effective transfer ratio", effectiveTransferRatio)
            addData("Effective slip ratio", effectiveSlipRatio)
        }
    }

    fun readyToShoot() = (error <= 100.0)

    fun toggleBangBang() {
        bangbang = !bangbang
    }
}