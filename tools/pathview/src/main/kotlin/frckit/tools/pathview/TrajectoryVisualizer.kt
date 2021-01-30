package frckit.tools.pathview

import edu.wpi.first.wpilibj.trajectory.Trajectory
import java.awt.event.WindowAdapter
import java.awt.event.WindowEvent
import javax.swing.*
import kotlin.system.exitProcess
import edu.wpi.first.wpilibj.geometry.Rotation2d

import edu.wpi.first.wpilibj.geometry.Pose2d

import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics

import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator

import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint

/**
 * Creates a new trajectory visualizer, which can draw and display the ideal following of WPILib differential drive
 * motion profiles
 *
 * @param ppi The pixels per inch value to use for the simulation window
 * @param trajectory The trajectory object to display
 * @param trackWidthInches The track-width of the robot to display, in inches
 * @param markers A list of positions of markers to display on the field.  They appear as 7-inch diameter magenta circles (this is specific to 2021 challenges and is hard-coded).
 */
class TrajectoryVisualizer(val ppi: Double, val trajectory: Trajectory, val trackWidthInches: Double, val markers: List<Translation2d> = listOf()) {
    private val frame = JFrame("Trajectory Visualization")

    fun start() {
        frame.addWindowListener(object : WindowAdapter() {
            override fun windowClosing(e: WindowEvent?) {
                exitProcess(0) //Force the application to close (this is needed to stop the executor)
            }
        })

        val canvas = TrajectoryViewCanvas(ppi, 250.0, 250.0, trajectory, trackWidthInches, markers)

        val buttonPanel = JPanel()
        buttonPanel.layout = BoxLayout(buttonPanel, BoxLayout.X_AXIS)

        val playButton = JButton("Simulate")
        playButton.addActionListener { canvas.startSimulation() }
        val resetButton = JButton("Reset")
        resetButton.addActionListener { canvas.resetSimulation() }

        buttonPanel.add(playButton)
        buttonPanel.add(resetButton)

        frame.layout = BoxLayout(frame.contentPane, BoxLayout.Y_AXIS)

        frame.contentPane.add(canvas)
        frame.contentPane.add(buttonPanel)

        SwingUtilities.invokeLater {
            frame.pack()
            frame.isVisible = true
        }
    }
}

//Testing main function, not for use
/*
fun main() {
    // Create config for trajectory
    // Create config for trajectory
    val config: TrajectoryConfig = TrajectoryConfig(
        120.0,
        50.0
    ) // Add kinematics to ensure max speed is actually obeyed
    config.setKinematics(DifferentialDriveKinematics(24.0))
    config.addConstraint(CentripetalAccelerationConstraint(150.0))

    // An example trajectory to follow.  All units in meters.

    // An example trajectory to follow.  All units in meters.
    val waypoints = listOf(
        Pose2d(30.0, 30.0, Rotation2d()),
        Pose2d(90.0, 60.0, Rotation2d.fromDegrees(45.0)),
        Pose2d(180.0, 90.0, Rotation2d()),
        Pose2d(270.0, 60.0, Rotation2d.fromDegrees(-45.0)),
        Pose2d(300.0, 30.0, Rotation2d()),
        Pose2d(330.0, 60.0, Rotation2d.fromDegrees(90.0)),
        Pose2d(300.0, 90.0, Rotation2d.fromDegrees(180.0)),
        Pose2d(270.0, 60.0, Rotation2d.fromDegrees(-90.0 - 45.0)),
        Pose2d(180.0, 30.0, Rotation2d.fromDegrees(-180.0)),
        Pose2d(90.0, 60.0, Rotation2d.fromDegrees(90.0 + 45.0)),
        Pose2d(30.0, 90.0, Rotation2d.fromDegrees(180.0))
    )

    val interiorList = listOf(
        Translation2d(90.0, 60.0),
        Translation2d(180.0, 90.0),
        Translation2d(260.0, 60.0),
        Translation2d(300.0, 30.0),
        Translation2d(340.0, 60.0),
        Translation2d(300.0, 90.0),
        Translation2d(260.0, 60.0),
        Translation2d(180.0, 30.0),
        Translation2d(90.0, 60.0)
    )

    val start = Pose2d(30.0, 30.0, Rotation2d())
    val end = Pose2d(30.0, 90.0, Rotation2d.fromDegrees(180.0))


    /*


    val exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        Pose2d(30.0, 30.0, Rotation2d()),
        listOf(
            Translation2d(75.0, 30.0), Translation2d(105.0, 90.0), Translation2d(255.0, 90.0),
            Translation2d(285.0, 30.0), Translation2d(330.0, 40.0), Translation2d(330.0, 80.0),
            Translation2d(285.0, 90.0), Translation2d(255.0, 30.0), Translation2d(105.0, 30.0),
            Translation2d(75.0, 90.0)
        ),
        Pose2d(30.0, 90.0, Rotation2d.fromDegrees(-180.0)),
        config
    )


    */

    val exampleTrajectory = TrajectoryGenerator.generateTrajectory(waypoints, config)
    //val exampleTrajectory = TrajectoryGenerator.generateTrajectory(start, interiorList, end, config)

    val visualizer = TrajectoryVisualizer(2.5, exampleTrajectory, 24.0, listOf(
        Translation2d(60.0, 60.0),
        Translation2d(120.0, 60.0),
        Translation2d(150.0, 60.0),
        Translation2d(180.0, 60.0),
        Translation2d(210.0, 60.0),
        Translation2d(240.0, 60.0),
        Translation2d(300.0, 60.0)
    ))
    visualizer.start()
}

 */