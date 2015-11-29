package net.sf.jaer;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.SocketChannel;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import org.apache.commons.validator.routines.InetAddressValidator;
import org.apache.commons.validator.routines.IntegerValidator;

import javafx.application.Application;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.LongProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.property.SimpleLongProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.event.EventHandler;
import javafx.scene.control.TabPane;
import javafx.scene.control.TextField;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import net.sf.jaer.jaerfx2.GUISupport;
import net.sf.jaer.jaerfx2.SSHS.SSHSType;

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

	public static class caerControlResponse {
		private final caerControlConfigAction action;
		private final SSHSType type;
		private final String message;

		public caerControlResponse(final byte action, final byte type, final String message) {
			this.action = caerControlConfigAction.getActionByCode(action);
			this.type = SSHSType.getTypeByCode(type);
			this.message = message;
		}

		public caerControlConfigAction getAction() {
			return (action);
		}

		public SSHSType getType() {
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

	@Override
	public void start(final Stage primaryStage) {
		if (GUISupport.checkJavaVersion(primaryStage)) {
			final VBox gui = new VBox(20);
			gui.getChildren().add(connectToCaerControlServerGUI());
			gui.getChildren().add(mainGUI);

			GUISupport.startGUI(primaryStage, gui, "cAER Control Utility (JavaFX GUI)",
				new EventHandler<WindowEvent>() {
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
		}
	}

	private HBox connectToCaerControlServerGUI() {
		final HBox connectToCaerControlServerGUI = new HBox(10);

		// IP address to connect to.
		final TextField ipAddress = GUISupport.addTextField(null, "127.0.0.1");
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
		final TextField port = GUISupport.addTextField(null, "4040");
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

		if ((keyResponse.getAction() != caerControlConfigAction.ERROR) && (keyResponse.getType() == SSHSType.STRING)) {
			// For each key, query its value type and then its actual value, and
			// build up the proper configuration control knob.
			for (final String key : keyResponse.getMessage().split("\0")) {
				// Query type.
				sendCommand(caerControlConfigAction.GET_TYPES, node, key, null, null);

				// Read and parse response.
				final caerControlResponse typeResponse = readResponse();

				if ((typeResponse.getAction() != caerControlConfigAction.ERROR)
					&& (typeResponse.getType() == SSHSType.STRING)) {
					// For each key and type, we get the current value and then
					// build the configuration control knob.
					for (final String type : typeResponse.getMessage().split("\0")) {
						// Query current value.
						sendCommand(caerControlConfigAction.GET, node, key, SSHSType.getTypeByName(type), null);

						// Read and parse response.
						final caerControlResponse valueResponse = readResponse();

						if ((valueResponse.getAction() != caerControlConfigAction.ERROR)
							&& (valueResponse.getType() != SSHSType.UNKNOWN)) {
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
			&& (childResponse.getType() == SSHSType.STRING)) {
			// Split up message containing child nodes by using the NUL
			// terminator as separator.
			final TabPane tabPane = new TabPane();
			contentPane.getChildren().add(tabPane);

			for (final String childNode : childResponse.getMessage().split("\0")) {
				GUISupport.addTab(tabPane, populateInterface(node + childNode + "/"), childNode);
			}
		}

		return (contentPane);
	}

	private Pane generateConfigGUI(final String node, final String key, final String type, final String value) {
		final HBox configBox = new HBox(20);

		GUISupport.addLabel(configBox, String.format("%s (%s):", key, type), null);

		switch (SSHSType.getTypeByName(type)) {
			case BOOL:
				GUISupport.addCheckBox(configBox, null, value.equals("true")).selectedProperty()
					.addListener(new ChangeListener<Boolean>() {
						@SuppressWarnings("unused")
						@Override
						public void changed(final ObservableValue<? extends Boolean> observable, final Boolean oldValue,
							final Boolean newValue) {
							// Send new value to cAER control server.
							sendCommand(caerControlConfigAction.PUT, node, key, SSHSType.BOOL,
								(newValue) ? ("true") : ("false"));
							readResponse();
						}
					});
				break;

			case BYTE: {
				final IntegerProperty backendValue = new SimpleIntegerProperty(Byte.parseByte(value));
				GUISupport.addTextNumberField(configBox, backendValue, 0, Byte.MAX_VALUE);
				backendValue.addListener(new ChangeListener<Number>() {
					@SuppressWarnings("unused")
					@Override
					public void changed(final ObservableValue<? extends Number> observable, final Number oldValue,
						final Number newValue) {
						// Send new value to cAER control server.
						sendCommand(caerControlConfigAction.PUT, node, key, SSHSType.BYTE,
							Byte.toString(newValue.byteValue()));
						readResponse();
					}
				});
				GUISupport.addSlider(configBox, backendValue.get(), 0, Byte.MAX_VALUE).valueProperty()
					.bindBidirectional(backendValue);
				break;
			}

			case SHORT: {
				final IntegerProperty backendValue = new SimpleIntegerProperty(Short.parseShort(value));
				GUISupport.addTextNumberField(configBox, backendValue, 0, Short.MAX_VALUE);
				backendValue.addListener(new ChangeListener<Number>() {
					@SuppressWarnings("unused")
					@Override
					public void changed(final ObservableValue<? extends Number> observable, final Number oldValue,
						final Number newValue) {
						// Send new value to cAER control server.
						sendCommand(caerControlConfigAction.PUT, node, key, SSHSType.SHORT,
							Short.toString(newValue.shortValue()));
						readResponse();
					}
				});
				GUISupport.addSlider(configBox, backendValue.get(), 0, Short.MAX_VALUE).valueProperty()
					.bindBidirectional(backendValue);
				break;
			}

			case INT: {
				final IntegerProperty backendValue = new SimpleIntegerProperty(Integer.parseInt(value));
				GUISupport.addTextNumberField(configBox, backendValue, 0, Integer.MAX_VALUE);
				backendValue.addListener(new ChangeListener<Number>() {
					@SuppressWarnings("unused")
					@Override
					public void changed(final ObservableValue<? extends Number> observable, final Number oldValue,
						final Number newValue) {
						// Send new value to cAER control server.
						sendCommand(caerControlConfigAction.PUT, node, key, SSHSType.INT,
							Integer.toString(newValue.intValue()));
						readResponse();
					}
				});
				GUISupport.addSlider(configBox, backendValue.get(), 0, Integer.MAX_VALUE).valueProperty()
					.bindBidirectional(backendValue);
				break;
			}

			case LONG: {
				final LongProperty backendValue = new SimpleLongProperty(Long.parseLong(value));
				GUISupport.addTextNumberField(configBox, backendValue, 0, Long.MAX_VALUE);
				backendValue.addListener(new ChangeListener<Number>() {
					@SuppressWarnings("unused")
					@Override
					public void changed(final ObservableValue<? extends Number> observable, final Number oldValue,
						final Number newValue) {
						// Send new value to cAER control server.
						sendCommand(caerControlConfigAction.PUT, node, key, SSHSType.LONG,
							Long.toString(newValue.longValue()));
						readResponse();
					}
				});
				GUISupport.addSlider(configBox, backendValue.get(), 0, Long.MAX_VALUE).valueProperty()
					.bindBidirectional(backendValue);
				break;
			}

			case FLOAT: {
				final DoubleProperty backendValue = new SimpleDoubleProperty(Float.parseFloat(value));
				GUISupport.addTextNumberField(configBox, backendValue, Float.MIN_VALUE, Float.MAX_VALUE);
				backendValue.addListener(new ChangeListener<Number>() {
					@SuppressWarnings("unused")
					@Override
					public void changed(final ObservableValue<? extends Number> observable, final Number oldValue,
						final Number newValue) {
						// Send new value to cAER control server.
						sendCommand(caerControlConfigAction.PUT, node, key, SSHSType.FLOAT,
							Float.toString(newValue.floatValue()));
						readResponse();
					}
				});
				GUISupport.addSlider(configBox, backendValue.get(), Float.MIN_VALUE, Float.MAX_VALUE).valueProperty()
					.bindBidirectional(backendValue);
				break;
			}

			case DOUBLE: {
				final DoubleProperty backendValue = new SimpleDoubleProperty(Double.parseDouble(value));
				GUISupport.addTextNumberField(configBox, backendValue, Double.MIN_VALUE, Double.MAX_VALUE);
				backendValue.addListener(new ChangeListener<Number>() {
					@SuppressWarnings("unused")
					@Override
					public void changed(final ObservableValue<? extends Number> observable, final Number oldValue,
						final Number newValue) {
						// Send new value to cAER control server.
						sendCommand(caerControlConfigAction.PUT, node, key, SSHSType.DOUBLE,
							Double.toString(newValue.doubleValue()));
						readResponse();
					}
				});
				GUISupport.addSlider(configBox, backendValue.get(), Double.MIN_VALUE, Double.MAX_VALUE).valueProperty()
					.bindBidirectional(backendValue);
				break;
			}

			case STRING:
				GUISupport.addTextField(configBox, value).textProperty().addListener(new ChangeListener<String>() {
					@SuppressWarnings("unused")
					@Override
					public void changed(final ObservableValue<? extends String> observable, final String oldValue,
						final String newValue) {
						// Send new value to cAER control server.
						sendCommand(caerControlConfigAction.PUT, node, key, SSHSType.STRING, newValue);
						readResponse();
					}
				});
				break;

			default:
				// Unknown value type, just display the returned string.
				GUISupport.addLabel(configBox, value, null);
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
		final SSHSType type, final String value) {
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
