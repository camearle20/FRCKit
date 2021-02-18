package frckit.tools.pathview

import edu.wpi.first.wpilibj.geometry.Translation2d
import java.awt.Color

data class TrajectoryMarker @JvmOverloads constructor(val position: Translation2d, val diameterMeters: Double, val color: Color = Color.MAGENTA)