package ca.frc6390.athena.mechanism.core;

/**
 * Runtime context visible to rule predicates.
 */
public interface RuleContext {
    AxisState axis(AxisRef axis);

    OutputRequest request();

    default OutputRequest request(AxisRef axis) {
        OutputRequest request = request();
        if (request != null && request.axis() == axis) {
            return request;
        }
        return OutputRequest.of(axis, Outputs.neutral());
    }
}
