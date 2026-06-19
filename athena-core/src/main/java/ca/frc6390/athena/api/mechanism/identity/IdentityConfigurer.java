package ca.frc6390.athena.api.mechanism.identity;

@FunctionalInterface
public interface IdentityConfigurer {
    IdentityConfig apply(IdentityConfig identity);
}
