package org.firstinspires.ftc.teamcode.subsystems

import club.bitflip.utils.hardware.MotorEx
import club.bitflip.utils.hardware.ServoEx
import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.InterpolatedLookupTable

@Configurable
class Shooter(hardwareMap: HardwareMap, telemetry: Telemetry) : Subsystem(hardwareMap, telemetry) {

    companion object {
        @JvmField
        var gain = 0.01
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

    private val velocityLookupTable = InterpolatedLookupTable(
        distanceArray,
        speedArray
    )

    private val timeInAirLookupTable = InterpolatedLookupTable(
        distanceArray,
        timeInAirArray
    )

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
        timeInAir = timeInAirLookupTable.calculate(distance)

        targetFlywheelRPM = velocityLookupTable.calculate(distance)
    }

    override fun loop() {
        controlLoop()

        telemetry.run {
            addData("Target flywheel RPM", targetFlywheelRPM)
            addData("Actual flywheel RPM", flywheelRPM)
            addData("Motor power", output)
        }
    }

    fun readyToShoot() = (error <= 100.0)

    fun toggleBangBang() {
        bangbang = !bangbang
    }
}