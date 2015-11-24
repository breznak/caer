package net.sf.jaer;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.SocketChannel;
import java.util.Collections;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

import org.apache.commons.validator.routines.InetAddressValidator;
import org.apache.commons.validator.routines.IntegerValidator;

import javafx.application.Application;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.event.EventHandler;
import javafx.geometry.Rectangle2D;
import javafx.scene.Scene;
import javafx.scene.control.ScrollPane;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import javafx.scene.control.TextField;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.stage.Screen;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import net.sf.jaer.Numbers.NumberFormat;
import net.sf.jaer.Numbers.NumberOptions;

public final class caerControlGuiJavaFX extends Application {
	public static enum caerControlConfigAction {
		NODE_EXISTS(0),
		ATTR_EXISTS(1),
		GET(2),
		PUT(3),
		ERROR(4),
		GET_CHILDREN(5),
		GET_ATTRIBUTES(6),
		GET_TYPES(7);

		private final byte code;

		private static final Map<Byte, caerControlConfigAction> codeToEnumMap = Collections
			.unmodifiableMap(caerControlConfigAction.initCodeToEnumMap());

		private static Map<Byte, caerControlConfigAction> initCodeToEnumMap() {
			final Map<Byte, caerControlConfigAction> initCodeToEnumMap = new HashMap<>();
			for (final caerControlConfigAction type : caerControlConfigAction.values()) {
				initCodeToEnumMap.put(type.code, type);
			}
			return (initCodeToEnumMap);
		}

		caerControlConfigAction(final int code) {
			this.code = (byte) code;
		}

		public byte getCode() {
			return (code);
		}

		public static caerControlConfigAction getActionByCode(final byte code) {
			if (caerControlConfigAction.codeToEnumMap.containsKey(code)) {
				return (caerControlConfigAction.codeToEnumMap.get(code));
			}

			return (null);
		}
	}

	public static enum caerControlConfigType {
		UNKNOWN(-1, null),
		BOOL(0, "bool"),
		BYTE(1, "byte"),
		SHORT(2, "short"),
		INT(3, "int"),
		LONG(4, "long"),
		FLOAT(5, "float"),
		DOUBLE(6, "double"),
		STRING(7, "string");

		private final byte code;
		private final String name;

		private static final Map<String, caerControlConfigType> nameToEnumMap = Collections
			.unmodifiableMap(caerControlConfigType.initNameToEnumMap());
		private static final Map<Byte, caerControlConfigType> codeToEnumMap = Collections
			.unmodifiableMap(caerControlConfigType.initCodeToEnumMap());

		private static Map<String, caerControlConfigType> initNameToEnumMap() {
			final Map<String, caerControlConfigType> initNameToEnumMap = new HashMap<>();
			for (final caerControlConfigType type : caerControlConfigType.values()) {
				initNameToEnumMap.put(type.name, type);
			}
			return (initNameToEnumMap);
		}

		private static Map<Byte, caerControlConfigType> initCodeToEnumMap() {
			final Map<Byte, caerControlConfigType> initCodeToEnumMap = new HashMap<>();
			for (final caerControlConfigType type : caerControlConfigType.values()) {
				initCodeToEnumMap.put(type.code, type);
			}
			return (initCodeToEnumMap);
		}

		caerControlConfigType(final int code, final String name) {
			this.code = (byte) code;
			this.name = name;
		}

		public byte getCode() {
			return (code);
		}

		public String getName() {
			return (name);
		}

		public static caerControlConfigType getTypeByName(final String name) {
			if (caerControlConfigType.nameToEnumMap.containsKey(name)) {
				return (caerControlConfigType.nameToEnumMap.get(name));
			}

			return (null);
		}

		public static caerControlConfigType getTypeByCode(final byte code) {
			if (caerControlConfigType.codeToEnumMap.containsKey(code)) {
				return (caerControlConfigType.codeToEnumMap.get(code));
			}

			return (null);
		}
	}

	public static class caerControlResponse {
		private final caerControlConfigAction action;
		private final caerControlConfigType type;
		private final String message;

		public caerControlResponse(final byte action, final byte type, final String message) {
			this.action = caerControlConfigAction.getActionByCode(action);
			this.type = caerControlConfigType.getTypeByCode(type);
			this.message = message;
		}

		public caerControlConfigAction getAction() {
			return (action);
		}

		public caerControlConfigType getType() {
			return (type);
		}

		public String getMessage() {
			return (message);
		}
	}

	private final VBox mainGUI = new VBox(20);
	private SocketChannel controlSocket = null;
	private final ByteBuffer netBuf = ByteBuffer.allocateDirect(1024).order(ByteOrder.LITTLE_ENDIAN);

	public static void main(final String[] args) {
		// Launch the JavaFX application: do initialization and call start()
		// when ready.
		Application.launch(args);
	}

	private static void addToTab(final TabPane tabPane, final Pane content, final String name) {
		final Tab t = new Tab(name);

		if (content != null) {
			final ScrollPane p = new ScrollPane(content);

			p.setFitToWidth(true);
			p.setPrefHeight(Screen.getPrimary().getVisualBounds().getHeight());
			// TODO: is there a better way to maximize the ScrollPane content?

			t.setContent(p);
		}

		t.setClosable(false);

		tabPane.getTabs().add(t);
	}

	@Override
	public void start(final Stage primaryStage) {
		GUISupport.checkJavaVersion(primaryStage);

		final VBox gui = new VBox(20);
		gui.getChildren().add(connectToCaerControlServerGUI());
		gui.getChildren().add(mainGUI);

		final BorderPane main = new BorderPane();
		main.setCenter(gui);

		final Rectangle2D screen = Screen.getPrimary().getVisualBounds();
		final Scene rootScene = new Scene(main, screen.getWidth(), screen.getHeight(), Color.GRAY);

		primaryStage.setTitle("cAER Control Utility (JavaFX GUI)");
		primaryStage.setScene(rootScene);

		primaryStage.setOnCloseRequest(new EventHandler<WindowEvent>() {
			@Override
			public void handle(@SuppressWarnings("unused") final WindowEvent evt) {
				try {
					cleanupConnection();
				}
				catch (final IOException e) {
					// Ignore this on closing.
				}
			}
		});

		primaryStage.show();
	}

	private HBox connectToCaerControlServerGUI() {
		final HBox connectToCaerControlServerGUI = new HBox(10);

		// IP address to connect to.
		final TextField ipAddress = GUISupport.addTextField(null, "127.0.0.1", null);
		GUISupport.addLabelWithControlsHorizontal(connectToCaerControlServerGUI, "IP address:",
			"Enter the IP address of the cAER control server.", ipAddress);

		ipAddress.textProperty().addListener(new ChangeListener<String>() {
			@SuppressWarnings("unused")
			@Override
			public void changed(final ObservableValue<? extends String> observable, final String oldValue,
				final String newValue) {
				// Validate IP address.
				final InetAddressValidator ipValidator = InetAddressValidator.getInstance();

				if (ipValidator.isValidInet4Address(newValue)) {
					ipAddress.setStyle("");
				}
				else {
					ipAddress.setStyle("-fx-background-color: #FF5757");
				}
			}
		});

		// Port to connect to.
		final TextField port = GUISupport.addTextField(null, "4040", null);
		GUISupport.addLabelWithControlsHorizontal(connectToCaerControlServerGUI, "Port:",
			"Enter the port of the cAER control server.", port);

		port.textProperty().addListener(new ChangeListener<String>() {
			@SuppressWarnings("unused")
			@Override
			public void changed(final ObservableValue<? extends String> observable, final String oldValue,
				final String newValue) {
				// Validate port.
				final IntegerValidator portValidator = IntegerValidator.getInstance();

				if (portValidator.isInRange(Integer.parseInt(newValue), 1, 65535)) {
					port.setStyle("");
				}
				else {
					port.setStyle("-fx-background-color: #FF5757");
				}
			}
		});

		// Connect button.
		GUISupport.addButtonWithMouseClickedHandler(connectToCaerControlServerGUI, "Connect", true, null,
			new EventHandler<MouseEvent>() {
				@SuppressWarnings("unused")
				@Override
				public void handle(final MouseEvent event) {
					try {
						cleanupConnection();

						controlSocket = SocketChannel
							.open(new InetSocketAddress(ipAddress.getText(), Integer.parseInt(port.getText())));

						mainGUI.getChildren().add(populateInterface("/"));
					}
					catch (NumberFormatException | IOException e) {
						GUISupport.showDialogException(e);
					}
				}
			});

		return (connectToCaerControlServerGUI);
	}

	// Add tabs recursively with configuration values to explore.
	private Pane populateInterface(final String node) {
		final VBox contentPane = new VBox(20);

		// Add all keys at this level.
		// First query what they are.
		sendCommand(caerControlConfigAction.GET_ATTRIBUTES, node, null, null, null);

		// Read and parse response.
		final caerControlResponse keyResponse = readResponse();

		if ((keyResponse.getAction() != caerControlConfigAction.ERROR)
			&& (keyResponse.getType() == caerControlConfigType.STRING)) {
			// For each key, query its value type and then its actual value, and
			// build up the proper configuration control knob.
			for (final String key : keyResponse.getMessage().split("\0")) {
				// Query type.
				sendCommand(caerControlConfigAction.GET_TYPES, node, key, null, null);

				// Read and parse response.
				final caerControlResponse typeResponse = readResponse();

				if ((typeResponse.getAction() != caerControlConfigAction.ERROR)
					&& (typeResponse.getType() == caerControlConfigType.STRING)) {
					// For each key and type, we get the current value and then
					// build the configuration control knob.
					for (final String type : typeResponse.getMessage().split("\0")) {
						// Query current value.
						sendCommand(caerControlConfigAction.GET, node, key, caerControlConfigType.getTypeByName(type),
							null);

						// Read and parse response.
						final caerControlResponse valueResponse = readResponse();

						if ((valueResponse.getAction() != caerControlConfigAction.ERROR)
							&& (valueResponse.getType() != caerControlConfigType.UNKNOWN)) {
							// This is the current value. We have everything.
							contentPane.getChildren()
								.add(generateConfigGUI(node, key, type, valueResponse.getMessage()));
						}
					}
				}
			}
		}

		// Then query available sub-nodes.
		sendCommand(caerControlConfigAction.GET_CHILDREN, node, null, null, null);

		// Read and parse response.
		final caerControlResponse childResponse = readResponse();

		if ((childResponse.getAction() != caerControlConfigAction.ERROR)
			&& (childResponse.getType() == caerControlConfigType.STRING)) {
			// Split up message containing child nodes by using the NUL
			// terminator as separator.
			final TabPane tabPane = new TabPane();
			contentPane.getChildren().add(tabPane);

			for (final String childNode : childResponse.getMessage().split("\0")) {
				caerControlGuiJavaFX.addToTab(tabPane, populateInterface(node + childNode + "/"), childNode);
			}
		}

		return (contentPane);
	}

	private Pane generateConfigGUI(final String node, final String key, final String type, final String value) {
		final HBox configBox = new HBox(20);

		GUISupport.addLabel(configBox, String.format("%s (%s):", key, type), null, null, null);

		switch (caerControlConfigType.getTypeByName(type)) {
			case BOOL:
				GUISupport.addCheckBox(configBox, null, value.equals("true")).selectedProperty()
					.addListener(new ChangeListener<Boolean>() {
						@SuppressWarnings("unused")
						@Override
						public void changed(final ObservableValue<? extends Boolean> observable, final Boolean oldValue,
							final Boolean newValue) {
							// Send new value to cAER control server.
							sendCommand(caerControlConfigAction.PUT, node, key,
								caerControlConfigType.getTypeByName(type), (newValue) ? ("true") : ("false"));
							readResponse();
						}
					});
				break;

			case BYTE:
				GUISupport
					.addTextNumberField(configBox, Integer.parseInt(value), 3, 0, 255, NumberFormat.DECIMAL,
						EnumSet.of(NumberOptions.UNSIGNED), null)
					.textProperty().addListener(new ChangeListener<String>() {
						@SuppressWarnings("unused")
						@Override
						public void changed(ObservableValue<? extends String> observable, String oldValue,
							String newValue) {
							// Send new value to cAER control server.
							sendCommand(caerControlConfigAction.PUT, node, key,
								caerControlConfigType.getTypeByName(type), newValue);
							readResponse();
						}
					});
				break;

			case SHORT:
				GUISupport
					.addTextNumberField(configBox, Integer.parseInt(value), 5, 0, 65535, NumberFormat.DECIMAL,
						EnumSet.of(NumberOptions.UNSIGNED), null)
					.textProperty().addListener(new ChangeListener<String>() {
						@SuppressWarnings("unused")
						@Override
						public void changed(ObservableValue<? extends String> observable, String oldValue,
							String newValue) {
							// Send new value to cAER control server.
							sendCommand(caerControlConfigAction.PUT, node, key,
								caerControlConfigType.getTypeByName(type), newValue);
							readResponse();
						}
					});
				break;

			case INT:
				GUISupport
					.addTextNumberField(configBox, Long.parseLong(value), 10, 0, 4294967295L, NumberFormat.DECIMAL,
						EnumSet.of(NumberOptions.UNSIGNED), null)
					.textProperty().addListener(new ChangeListener<String>() {
						@SuppressWarnings("unused")
						@Override
						public void changed(ObservableValue<? extends String> observable, String oldValue,
							String newValue) {
							// Send new value to cAER control server.
							sendCommand(caerControlConfigAction.PUT, node, key,
								caerControlConfigType.getTypeByName(type), newValue);
							readResponse();
						}
					});
				break;

			case LONG:
				GUISupport
					.addTextNumberField(configBox, Long.parseLong(value), 19, 0, Long.MAX_VALUE, NumberFormat.DECIMAL,
						EnumSet.of(NumberOptions.UNSIGNED), null)
					.textProperty().addListener(new ChangeListener<String>() {
						@SuppressWarnings("unused")
						@Override
						public void changed(ObservableValue<? extends String> observable, String oldValue,
							String newValue) {
							// Send new value to cAER control server.
							sendCommand(caerControlConfigAction.PUT, node, key,
								caerControlConfigType.getTypeByName(type), newValue);
							readResponse();
						}
					});
				break;

			case FLOAT:

				break;

			case DOUBLE:

				break;

			case STRING:
				GUISupport.addTextField(configBox, value, null).textProperty()
					.addListener(new ChangeListener<String>() {
						@SuppressWarnings("unused")
						@Override
						public void changed(ObservableValue<? extends String> observable, String oldValue,
							String newValue) {
							// Send new value to cAER control server.
							sendCommand(caerControlConfigAction.PUT, node, key,
								caerControlConfigType.getTypeByName(type), newValue);
							readResponse();
						}
					});
				break;

			default:
				// Unknown value type, just display the returned string.
				GUISupport.addLabel(configBox, value, null, null, null);
				break;
		}

		return (configBox);
	}

	public static boolean writeBuffer(final ByteBuffer buf, final SocketChannel chan) {
		final int toWrite = buf.remaining();
		int written = 0;

		while (written < toWrite) {
			try {
				written += chan.write(buf);
			}
			catch (final IOException e) {
				GUISupport.showDialogException(e);
				return (false);
			}
		}

		return (true);
	}

	public static boolean readBuffer(final ByteBuffer buf, final SocketChannel chan) {
		final int toRead = buf.remaining();
		int read = 0;

		while (read < toRead) {
			try {
				read += chan.read(buf);
			}
			catch (final IOException e) {
				GUISupport.showDialogException(e);
				return (false);
			}
		}

		return (true);
	}

	private void cleanupConnection() throws IOException {
		mainGUI.getChildren().clear();

		if (controlSocket != null) {
			controlSocket.close();
			controlSocket = null;
		}
	}

	private void sendCommand(final caerControlConfigAction action, final String node, final String key,
		final caerControlConfigType type, final String value) {
		netBuf.clear();

		// Fill buffer with values.
		// First 1 byte action.
		netBuf.put(action.getCode());

		// Second 1 byte type.
		if (type != null) {
			netBuf.put(type.getCode());
		}
		else {
			netBuf.put((byte) 0);
		}

		// Third 2 bytes extra length (currently unused).
		netBuf.putShort((short) 0);

		// Fourth 2 bytes node length.
		if (node != null) {
			netBuf.putShort((short) (node.length() + 1));
			// +1 for terminating NUL byte.
		}
		else {
			netBuf.putShort((short) 0);
		}

		// Fifth 2 bytes key length.
		if (key != null) {
			netBuf.putShort((short) (key.length() + 1));
			// +1 for terminating NUL byte.
		}
		else {
			netBuf.putShort((short) 0);
		}

		// Sixth 2 bytes value length.
		if (value != null) {
			netBuf.putShort((short) (value.length() + 1));
			// +1 for terminating NUL byte.
		}
		else {
			netBuf.putShort((short) 0);
		}

		// At this point the header should be complete at 10 bytes.
		assert (netBuf.position() == 10);

		// Now we concatenate the node, key and value strings, separated by
		// their
		// terminating NUL byte.
		if (node != null) {
			netBuf.put(node.getBytes());
			netBuf.put((byte) 0); // Terminating NUL byte.
		}
		if (key != null) {
			netBuf.put(key.getBytes());
			netBuf.put((byte) 0); // Terminating NUL byte.
		}
		if (value != null) {
			netBuf.put(value.getBytes());
			netBuf.put((byte) 0); // Terminating NUL byte.
		}

		netBuf.flip();

		caerControlGuiJavaFX.writeBuffer(netBuf, controlSocket);
	}

	private caerControlResponse readResponse() {
		netBuf.clear();

		// Get response header.
		netBuf.limit(4);
		caerControlGuiJavaFX.readBuffer(netBuf, controlSocket);

		// Get response message part.
		final short messageLength = netBuf.getShort(2);

		netBuf.limit(4 + messageLength);
		caerControlGuiJavaFX.readBuffer(netBuf, controlSocket);

		netBuf.flip();

		// Parse header.
		final byte action = netBuf.get();
		final byte type = netBuf.get();

		// Parse message.
		final byte[] messageBytes = new byte[messageLength - 1];
		// -1 because we don't care about the last NUL terminator.

		netBuf.position(4);
		netBuf.get(messageBytes);

		final String message = new String(messageBytes);

		return (new caerControlResponse(action, type, message));
	}
}
