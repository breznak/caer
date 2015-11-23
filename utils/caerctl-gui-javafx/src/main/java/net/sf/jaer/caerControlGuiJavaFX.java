package net.sf.jaer;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.SocketChannel;

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

		caerControlConfigAction(int code) {
			this.code = (byte) code;
		}

		public byte getCode() {
			return (this.code);
		}
	}

	public static enum caerControlConfigType {
		UNKNOWN(-1),
		BOOL(0),
		BYTE(1),
		SHORT(2),
		INT(3),
		LONG(4),
		FLOAT(5),
		DOUBLE(6),
		STRING(7);

		private final byte code;

		caerControlConfigType(int code) {
			this.code = (byte) code;
		}

		public byte getCode() {
			return (code);
		}
	}

	public static class caerControlResponse {
		private final byte action;
		private final byte type;
		private final String message;

		public caerControlResponse(byte action, byte type, String message) {
			this.action = action;
			this.type = type;
			this.message = message;
		}

		public byte getAction() {
			return (action);
		}

		public byte getType() {
			return (type);
		}

		public String getMessage() {
			return (message);
		}
	}

	private final TabPane tabLayout = new TabPane();
	private SocketChannel controlSocket = null;
	private final ByteBuffer netBuf = ByteBuffer.allocateDirect(1024);

	public static void main(final String[] args) {
		// Launch the JavaFX application: do initialization and call start()
		// when ready.
		Application.launch(args);
	}

	private static void addToTab(final TabPane tabPane, final Pane content, final String name) {
		final ScrollPane p = new ScrollPane(content);

		p.setFitToWidth(true);
		p.setPrefHeight(Screen.getPrimary().getVisualBounds().getHeight());
		// TODO: is there a better way to maximize the ScrollPane content?

		final Tab t = new Tab(name);

		t.setContent(p);
		t.setClosable(false);

		tabPane.getTabs().add(t);
	}

	@Override
	public void start(final Stage primaryStage) {
		GUISupport.checkJavaVersion(primaryStage);

		final VBox gui = new VBox(20);

		gui.getChildren().add(connectToCaerControlServerGUI());

		gui.getChildren().add(tabLayout);

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
				catch (IOException e) {
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
			public void changed(ObservableValue<? extends String> observable, String oldValue, String newValue) {
				// Validate IP address.
				InetAddressValidator ipValidator = InetAddressValidator.getInstance();

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
			public void changed(ObservableValue<? extends String> observable, String oldValue, String newValue) {
				// Validate port.
				IntegerValidator portValidator = IntegerValidator.getInstance();

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
				public void handle(MouseEvent event) {
					try {
						cleanupConnection();

						controlSocket = SocketChannel
							.open(new InetSocketAddress(ipAddress.getText(), Integer.parseInt(port.getText())));

						populateTabs();
					}
					catch (NumberFormatException | IOException e) {
						GUISupport.showDialogException(e);
					}
				}
			});

		return (connectToCaerControlServerGUI);
	}

	// Add tabs recursively with configuration values to explore.
	private void populateTabs() {
		// First query available nodes under the root.
		sendCommand(caerControlConfigAction.GET_CHILDREN, "/", null, null, null);

		readResponse();
	}

	public static boolean writeBuffer(ByteBuffer buf, SocketChannel chan) {
		int toWrite = buf.limit();
		int written = 0;

		while (written < toWrite) {
			try {
				written += chan.write(buf);
			}
			catch (IOException e) {
				GUISupport.showDialogException(e);
				return (false);
			}
		}

		return (true);
	}

	public static boolean readBuffer(ByteBuffer buf, SocketChannel chan) {
		int toRead = buf.limit();
		int read = 0;

		while (read < toRead) {
			try {
				read += chan.read(buf);
			}
			catch (IOException e) {
				GUISupport.showDialogException(e);
				return (false);
			}
		}

		return (true);
	}

	private void cleanupConnection() throws IOException {
		tabLayout.getTabs().clear();

		if (controlSocket != null) {
			controlSocket.close();
			controlSocket = null;
		}
	}

	private void sendCommand(caerControlConfigAction action, String node, String key, caerControlConfigType type,
		String value) {
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

		writeBuffer(netBuf, controlSocket);
	}

	private caerControlResponse readResponse() {
		netBuf.clear();

		// Get response header.
		netBuf.limit(4);
		readBuffer(netBuf, controlSocket);

		// Get response message part.
		short messageLength = netBuf.getShort(2);

		netBuf.limit(4 + messageLength);
		readBuffer(netBuf, controlSocket);

		netBuf.flip();

		// Parse header.
		byte action = netBuf.get();
		byte type = netBuf.get();

		// Parse message.
		byte[] messageBytes = new byte[messageLength];

		netBuf.position(4);
		netBuf.get(messageBytes);

		String message = new String(messageBytes);

		return (new caerControlResponse(action, type, message));
	}
}
