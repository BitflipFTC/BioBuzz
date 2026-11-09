package org.firstinspires.ftc.teamcode

import com.pedropathing.math.Pose

object OpModeStorage {
    public var savedPose: Pose? = null
    public var savedAlliance: Alliance? = null
}

enum class Alliance {
    RED,
    BLUE
}