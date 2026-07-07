package ca.frc6390.athena.drivetrain.swerve;

/**
 * Discoverable catalog of known swerve module base classes.
 */
public final class SwerveModules {
    private static final double FOUR_INCH_WHEEL_METERS = 0.1016;

    private SwerveModules() {
    }

    /**
     * Custom module base when no catalog module matches.
     */
    public abstract static class Custom extends SwerveModule {
        protected Custom() {
            super(SwerveModuleModel.custom(1.0, 1.0, FOUR_INCH_WHEEL_METERS));
        }

        protected Custom(SwerveModuleModel model) {
            super(model);
        }
    }

    /**
     * Swerve Drive Specialties modules.
     */
    public static final class SDS {
        private SDS() {
        }

        /**
         * SDS MK5N modules.
         */
        public static final class MK5N {
            private MK5N() {
            }

            /**
             * SDS MK5N ratio 3 module.
             */
            public abstract static class R3 extends SwerveModule {
                private static final SwerveModuleModel MODEL = new SwerveModuleModel(
                        "SDS",
                        "MK5N_R3",
                        5.36,
                        18.75,
                        FOUR_INCH_WHEEL_METERS);

                protected R3() {
                    super(MODEL);
                }
            }
        }
    }
}
