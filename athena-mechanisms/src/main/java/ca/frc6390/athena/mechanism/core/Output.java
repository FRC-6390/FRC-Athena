package ca.frc6390.athena.mechanism.core;

/**
 * Normalized hardware command that Athena can apply to a mechanism.
 */
public interface Output {
    /**
     * Percent output command.
     */
    interface Percent extends Output {
        /**
         * Returns requested percent output.
         *
         * @return percent output
         */
        double percent();
    }

    /**
     * Position command.
     */
    interface Position extends Output {
        /**
         * Returns requested position.
         *
         * @return position target
         */
        double position();
    }

    /**
     * Velocity command.
     */
    interface Velocity extends Output {
        /**
         * Returns requested velocity.
         *
         * @return velocity target
         */
        double velocity();
    }

    /**
     * Neutral/stop command.
     */
    interface Neutral extends Output {
    }

    /**
     * Fault command.
     */
    interface Fault extends Output {
        /**
         * Returns fault reason.
         *
         * @return reason
         */
        String reason();
    }
}
