package frc.robot;

import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.reflect.Field;

public class Compliance {
	@Retention(RetentionPolicy.RUNTIME)
	public static @interface FixMe {
		String reason() default "TBD";
	}

	public static class ComplianceError extends Error {
		public ComplianceError(FixMe fixme) {
			super(String.format("Code tagged as non-compliant: %s", fixme.reason()));
		}
	}

	@SuppressWarnings("unchecked")
	public static <T> T ensure(Class<?> object, String key) {
		Field field = null;
		try {
			field = object.getField(key);
		} catch (NoSuchFieldException | SecurityException e) {
			throw new Error(String.format("Field \"%s\" was not found in class \"%s\"", key, object));
		}
		FixMe annotation = field.getAnnotation(FixMe.class);
		if (annotation != null) {
			throw new ComplianceError(annotation);
		} else {
			try {
				return (T) field.get(null);
			} catch (IllegalArgumentException | IllegalAccessException e) {
				throw new Error(String.format("Field \"%s\" could not be accessed in class \"%s\"", key, object));
			}
		}
	}
}