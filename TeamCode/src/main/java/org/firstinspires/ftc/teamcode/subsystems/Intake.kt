package org.firstinspires.ftc.teamcode.subsystems

import club.bitflip.utils.hardware.MotorEx
import com.pedropathing.ivy.CommandBuilder
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.waitMs
import com.pedropathing.ivy.groups.Groups.sequential
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry

class Intake(hardwareMap: HardwareMap, telemetry: Telemetry) : Subsystem(hardwareMap, telemetry) {

    enum class Mode(val power: Double) {
        OFF(0.0),
        SHOOTING(0.8),
        ON(0.9),
        REVERSED(-0.5)
    }

    private val motorL = MotorEx("intakeL").zeroed().float().reverse()
    private val motorR = MotorEx("intakeR").zeroed().float()

    private val currentTimer = ElapsedTime()

    var reversed = false
    var mode = Mode.OFF
        set(newMode) {
            field = newMode
            motorL.power = newMode.power
            motorR.power = newMode.power
        }

    init {
        motorL.setCurrentAlert(2.0)
        currentTimer.reset()
    }

    var isOverCurrent = false

    override fun loop() {
        if (mode == Mode.ON) {
            if (reversed) mode = Mode.REVERSED
        }

//        if (!isOverCurrent && motorL.getCurrent() >= 1.8 && mode == Mode.ON) {
//            isOverCurrent = true
//            currentTimer.reset()
//        }
//
//        if (currentTimer.milliseconds() > 1500 && motorL.getCurrent() >= 2.1 && mode == Mode.ON && isOverCurrent) {
//            isOverCurrent = false;
//            mode = Mode.OFF
//        } else if (isOverCurrent && currentTimer.milliseconds() > 1500 && motorL.getCurrent() < 1.7) {
//            isOverCurrent = false
//        }
//
//        if (mode == Mode.REVERSED && !reversed) {
//            mode = Mode.ON
//        }

        telemetry.addData("Intake Mode", mode.name)
//        telemetry.addData("Intake current", motorL.getCurrent())
    }

    fun on() {
        mode = Mode.ON
    }

    fun transfer() {
        mode = Mode.SHOOTING
    }

    fun off() {
        mode = Mode.OFF
    }

    fun toggle() {
        mode = if (mode == Mode.ON) {
            Mode.OFF
        } else {
            Mode.ON
        }
    }
}