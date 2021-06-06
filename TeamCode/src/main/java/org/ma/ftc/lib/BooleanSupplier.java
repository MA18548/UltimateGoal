package org.ma.ftc.lib;

/**
 * Represents a supplier of boolean-valued results. This is the boolean-producing primitive specialization of {@link Supplier}.
 * There is no requirement that a new or distinct result be returned each time the supplier is invoked.
 * <p>
 * This is a functional interface whose functional method is {@link BooleanSupplier#getAsBoolean()}.
 * This is a feature of Java 8 that we don't have access to because of an outdated sdk.
 */
public class BooleanSupplier {

    /**
     * Gets a result.
     *
     * @return a result.
     */
    public boolean getAsBoolean() {
        return false;
    }
}
