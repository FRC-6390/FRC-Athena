package ca.frc6390.athena.drivetrain.swerve;

/**
 * Catalog of known swerve module base classes.
 */
public final class SwerveModules {
    private static final double THREE_INCH_WHEEL_METERS = inches(3.0);
    private static final double FOUR_INCH_WHEEL_METERS = inches(4.0);

    private SwerveModules() {
    }

    /**
     * Custom module base when no catalog module matches.
     */
    public static class Custom extends SwerveModule {
        public Custom() {
            super(SwerveModuleModel.custom(1.0, 1.0, FOUR_INCH_WHEEL_METERS));
        }

        public Custom(SwerveModuleModel model) {
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
         * SDS MK3 modules.
         */
        public static final class MK3 {
            private MK3() {
            }

            public static class STANDARD extends SwerveModule {
                public STANDARD() {
                    super(SwerveModules.model("SDS", "MK3_STANDARD", 8.16, 12.8, FOUR_INCH_WHEEL_METERS));
                }
            }

            public static class FAST extends SwerveModule {
                public FAST() {
                    super(SwerveModules.model("SDS", "MK3_FAST", 6.86, 12.8, FOUR_INCH_WHEEL_METERS));
                }
            }
        }

        /**
         * SDS MK4 modules.
         */
        public static final class MK4 {
            private MK4() {
            }

            public static class L1 extends SwerveModule {
                public L1() {
                    super(SwerveModules.model("SDS", "MK4_L1", 8.14, 12.8, FOUR_INCH_WHEEL_METERS));
                }
            }

            public static class L2 extends SwerveModule {
                public L2() {
                    super(SwerveModules.model("SDS", "MK4_L2", 6.75, 12.8, FOUR_INCH_WHEEL_METERS));
                }
            }

            public static class L3 extends SwerveModule {
                public L3() {
                    super(SwerveModules.model("SDS", "MK4_L3", 6.12, 12.8, FOUR_INCH_WHEEL_METERS));
                }
            }

            public static class L4 extends SwerveModule {
                public L4() {
                    super(SwerveModules.model("SDS", "MK4_L4", 5.14, 12.8, FOUR_INCH_WHEEL_METERS));
                }
            }
        }

        /**
         * SDS MK4i modules.
         */
        public static final class MK4I {
            private MK4I() {
            }

            public static class L1 extends SwerveModule {
                public L1() {
                    super(SwerveModules.model("SDS", "MK4I_L1", 8.14, 150.0 / 7.0, FOUR_INCH_WHEEL_METERS));
                }
            }

            public static class L2 extends SwerveModule {
                public L2() {
                    super(SwerveModules.model("SDS", "MK4I_L2", 6.75, 150.0 / 7.0, FOUR_INCH_WHEEL_METERS));
                }
            }

            public static class L3 extends SwerveModule {
                public L3() {
                    super(SwerveModules.model("SDS", "MK4I_L3", 6.12, 150.0 / 7.0, FOUR_INCH_WHEEL_METERS));
                }
            }

            public static class L1_PLUS extends SwerveModule {
                public L1_PLUS() {
                    super(SwerveModules.model("SDS", "MK4I_L1_PLUS", 7.13, 150.0 / 7.0, FOUR_INCH_WHEEL_METERS));
                }
            }

            public static class L2_PLUS extends SwerveModule {
                public L2_PLUS() {
                    super(SwerveModules.model("SDS", "MK4I_L2_PLUS", 5.9, 150.0 / 7.0, FOUR_INCH_WHEEL_METERS));
                }
            }

            public static class L3_PLUS extends SwerveModule {
                public L3_PLUS() {
                    super(SwerveModules.model("SDS", "MK4I_L3_PLUS", 5.36, 150.0 / 7.0, FOUR_INCH_WHEEL_METERS));
                }
            }
        }

        /**
         * SDS MK4n modules.
         */
        public static final class MK4N {
            private MK4N() {
            }

            public static class L1_PLUS extends SwerveModule {
                public L1_PLUS() {
                    super(SwerveModules.model("SDS", "MK4N_L1_PLUS", 7.13, 18.75, FOUR_INCH_WHEEL_METERS));
                }
            }

            public static class L2_PLUS extends SwerveModule {
                public L2_PLUS() {
                    super(SwerveModules.model("SDS", "MK4N_L2_PLUS", 5.9, 18.75, FOUR_INCH_WHEEL_METERS));
                }
            }

            public static class L3_PLUS extends SwerveModule {
                public L3_PLUS() {
                    super(SwerveModules.model("SDS", "MK4N_L3_PLUS", 5.36, 18.75, FOUR_INCH_WHEEL_METERS));
                }
            }
        }

        /**
         * SDS MK4c modules.
         */
        public static final class MK4C {
            private MK4C() {
            }

            public static class L1_PLUS extends SwerveModule {
                public L1_PLUS() {
                    super(SwerveModules.model("SDS", "MK4C_L1_PLUS", 7.13, 12.8, FOUR_INCH_WHEEL_METERS));
                }
            }

            public static class L2_PLUS extends SwerveModule {
                public L2_PLUS() {
                    super(SwerveModules.model("SDS", "MK4C_L2_PLUS", 5.9, 12.8, FOUR_INCH_WHEEL_METERS));
                }
            }

            public static class L3_PLUS extends SwerveModule {
                public L3_PLUS() {
                    super(SwerveModules.model("SDS", "MK4C_L3_PLUS", 5.36, 12.8, FOUR_INCH_WHEEL_METERS));
                }
            }
        }

        /**
         * SDS MK5n modules.
         */
        public static final class MK5N {
            private MK5N() {
            }

            public static class R1 extends SwerveModule {
                public R1() {
                    super(SwerveModules.model("SDS", "MK5N_R1", 7.03, 287.0 / 11.0, FOUR_INCH_WHEEL_METERS));
                }
            }

            public static class R2 extends SwerveModule {
                public R2() {
                    super(SwerveModules.model("SDS", "MK5N_R2", 6.03, 287.0 / 11.0, FOUR_INCH_WHEEL_METERS));
                }
            }

            public static class R3 extends SwerveModule {
                public R3() {
                    super(SwerveModules.model("SDS", "MK5N_R3", 5.27, 287.0 / 11.0, FOUR_INCH_WHEEL_METERS));
                }
            }
        }

        /**
         * SDS MK5i modules.
         */
        public static final class MK5I {
            private MK5I() {
            }

            public static class R1 extends SwerveModule {
                public R1() {
                    super(SwerveModules.model("SDS", "MK5I_R1", 7.03, 26.0, FOUR_INCH_WHEEL_METERS));
                }
            }

            public static class R2 extends SwerveModule {
                public R2() {
                    super(SwerveModules.model("SDS", "MK5I_R2", 6.03, 26.0, FOUR_INCH_WHEEL_METERS));
                }
            }

            public static class R3 extends SwerveModule {
                public R3() {
                    super(SwerveModules.model("SDS", "MK5I_R3", 5.27, 26.0, FOUR_INCH_WHEEL_METERS));
                }
            }
        }
    }

    /**
     * REV Robotics modules.
     */
    public static final class REV {
        private REV() {
        }

        /**
         * REV 3 inch MAXSwerve module speed options.
         */
        public static final class MAXSWERVE_3IN {
            private MAXSWERVE_3IN() {
            }

            public static class LOW extends SwerveModule {
                public LOW() {
                    super(SwerveModules.model("REV", "MAXSWERVE_3IN_LOW", 5.50, 9424.0 / 203.0, THREE_INCH_WHEEL_METERS));
                }
            }

            public static class MEDIUM extends SwerveModule {
                public MEDIUM() {
                    super(SwerveModules.model("REV", "MAXSWERVE_3IN_MEDIUM", 5.08, 9424.0 / 203.0, THREE_INCH_WHEEL_METERS));
                }
            }

            public static class HIGH extends SwerveModule {
                public HIGH() {
                    super(SwerveModules.model("REV", "MAXSWERVE_3IN_HIGH", 4.71, 9424.0 / 203.0, THREE_INCH_WHEEL_METERS));
                }
            }

            public static class EXTRA_HIGH_1 extends SwerveModule {
                public EXTRA_HIGH_1() {
                    super(SwerveModules.model("REV", "MAXSWERVE_3IN_EXTRA_HIGH_1", 4.50, 9424.0 / 203.0, THREE_INCH_WHEEL_METERS));
                }
            }

            public static class EXTRA_HIGH_2 extends SwerveModule {
                public EXTRA_HIGH_2() {
                    super(SwerveModules.model("REV", "MAXSWERVE_3IN_EXTRA_HIGH_2", 4.29, 9424.0 / 203.0, THREE_INCH_WHEEL_METERS));
                }
            }

            public static class EXTRA_HIGH_3 extends SwerveModule {
                public EXTRA_HIGH_3() {
                    super(SwerveModules.model("REV", "MAXSWERVE_3IN_EXTRA_HIGH_3", 4.00, 9424.0 / 203.0, THREE_INCH_WHEEL_METERS));
                }
            }

            public static class EXTRA_HIGH_4 extends SwerveModule {
                public EXTRA_HIGH_4() {
                    super(SwerveModules.model("REV", "MAXSWERVE_3IN_EXTRA_HIGH_4", 3.75, 9424.0 / 203.0, THREE_INCH_WHEEL_METERS));
                }
            }

            public static class EXTRA_HIGH_5 extends SwerveModule {
                public EXTRA_HIGH_5() {
                    super(SwerveModules.model("REV", "MAXSWERVE_3IN_EXTRA_HIGH_5", 3.56, 9424.0 / 203.0, THREE_INCH_WHEEL_METERS));
                }
            }
        }

        /**
         * REV 4 inch EasySwerve module presets.
         */
        public static final class EASYSWERVE_4IN {
            private EASYSWERVE_4IN() {
            }

            public static class STANDARD extends SwerveModule {
                public STANDARD() {
                    super(SwerveModules.model("REV", "EASYSWERVE_4IN_STANDARD", 6.3, 20.0, FOUR_INCH_WHEEL_METERS));
                }
            }
        }
    }

    private static SwerveModuleModel model(
            String vendor,
            String name,
            double driveReduction,
            double steerReduction,
            double wheelDiameterMeters) {
        return new SwerveModuleModel(vendor, name, driveReduction, steerReduction, wheelDiameterMeters);
    }

    private static double inches(double inches) {
        return inches * 0.0254;
    }
}
