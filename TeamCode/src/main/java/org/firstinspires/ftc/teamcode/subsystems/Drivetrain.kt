package org.firstinspires.ftc.teamcode.subsystems

import club.bitflip.utils.hardware.MotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.abs
import kotlin.math.max

class Drivetrain(hardwareMap: HardwareMap, telemetry: Telemetry) :
    Subsystem(hardwareMap, telemetry) {
    private val frontleft = MotorEx("frontleft").zeroed().brake().reverse()
    private val frontright = MotorEx("frontright").zeroed().brake()
    private val backleft = MotorEx("backleft").zeroed().brake().reverse()
    private val backright = MotorEx("backright").zeroed().brake()

    fun updateDrivetrain(x: Double, y: Double, rx: Double) {
        val max: Double
        var flPower: Double = (y + x + rx)
        var frPower: Double = (y - x - rx)
        var blPower: Double = (y - x + rx)
        var brPower: Double = (y + x - rx)

        max = max(abs(max(max(abs(frPower), abs(flPower)), abs(blPower))), abs(brPower))

        if (max > 1) {
            flPower /= max
            frPower /= max
            blPower /= max
            brPower /= max
        }

        frontleft.power = flPower
        frontright.power = frPower
        backleft.power = blPower
        backright.power = brPower
    }
}