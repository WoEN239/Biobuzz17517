package org.firstinspires.ftc.teamcode.utils

import android.content.Context
import com.acmerobotics.dashboard.FtcDashboard
import org.firstinspires.ftc.ftccommon.external.OnCreate

class DashboardSuppressor {
    companion object {
        @OnCreate
        @JvmStatic
        fun start(context: Context?) {
            FtcDashboard.start(context)
            FtcDashboard.suppressOpMode()
        }
    }
}