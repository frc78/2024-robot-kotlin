package frc.team78.lib

import edu.wpi.first.math.geometry.Translation2d

/**
 * This class contains the field elements for the 2024 game. When accessing an element, the alliance
 * is taken into account.
 */

/** The location of the speaker on the field. */
val SPEAKER_POSE by
    Alliance(Translation2d(0.meters, 5.5.meters), Translation2d(16.5.meters, 5.5.meters))

/** The location that a feed-shot should land on the field */
val PLOP_POSE by Alliance(Translation2d(0.meters, 7.meters), Translation2d(16.5.meters, 7.meters))

val SPEAKER_HEIGHT = 2.1.meters
