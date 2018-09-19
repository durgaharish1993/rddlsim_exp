package rddl;

public class NoStackTraceRuntimeException extends RuntimeException {
    @Override
    public synchronized Throwable fillInStackTrace() {
        return this;
    }
}
