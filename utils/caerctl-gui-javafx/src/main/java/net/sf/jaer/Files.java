package net.sf.jaer;

import java.io.File;
import java.util.List;

public class Files {
	public static boolean checkReadPermissions(final File f) {
		if (f == null) {
			throw new NullPointerException();
		}

		// We want to read, so it has to exist and be readable.
		if (f.exists() && f.isFile() && f.canRead()) {
			return true;
		}

		return false;
	}

	public static boolean checkWritePermissions(final File f) {
		if (f == null) {
			throw new NullPointerException();
		}

		if (f.exists() && f.isFile()) {
			if (f.canWrite()) {
				// If it exists already, but is writable.
				return true;
			}

			// Exists already, but is not writable.
			return false;
		}

		// Non-existing paths can usually be written to.
		return true;
	}

	public static boolean checkExtensions(final File f, final List<String> extensions) {
		for (final String ext : extensions) {
			if (f.getName().endsWith(ext.substring(ext.indexOf('.')))) {
				return true;
			}
		}

		return false;
	}
}
