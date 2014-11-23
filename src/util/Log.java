package util;

public class Log {
	
	public static void d(String message) {
		System.out.println(message);
	}
	
	
	public static void d(String tag, String message) {
		System.out.println("["+ tag + "] " + message);
	}
	
	
	public static void e(String message) {
		System.err.println(message);
	}
}
