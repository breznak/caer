package net.sf.jaer;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.SocketChannel;

import org.apache.commons.validator.routines.InetAddressValidator;
import org.apache.commons.validator.routines.IntegerValidator;

import javafx.application.Application;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.LongProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.event.EventHandler;
import javafx.geometry.Pos;
import javafx.scene.control.Label;
import javafx.scene.control.TabPane;
import javafx.scene.control.TextField;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import net.sf.jaer.jaerfx2.CAERCommunication;
import net.sf.jaer.jaerfx2.GUISupport;
import net.sf.jaer.jaerfx2.SSHS.SSHSAttributeValue;
import net.sf.jaer.jaerfx2.SSHS.SSHSType;

public final class caerControlGuiJavaFX extends Application {
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
		final TextField ipAddress = GUISupport.addTextField(null, CAERCommunication.DEFAULT_IP);
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
		final TextField port = GUISupport.addTextField(null, CAERCommunication.DEFAULT_PORT);
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
		CAERCommunication.sendCommand(controlSocket, netBuf, CAERCommunication.caerControlConfigAction.GET_ATTRIBUTES,
			node, null, null, null);

		// Read and parse response.
		final CAERCommunication.caerControlResponse keyResponse = CAERCommunication.readResponse(controlSocket, netBuf);

		if ((keyResponse.getAction() != CAERCommunication.caerControlConfigAction.ERROR)
			&& (keyResponse.getType() == SSHSType.STRING)) {
			// For each key, query its value type and then its actual value, and
			// build up the proper configuration control knob.
			for (final String key : keyResponse.getMessage().split("\0")) {
				// Query type.
				CAERCommunication.sendCommand(controlSocket, netBuf,
					CAERCommunication.caerControlConfigAction.GET_TYPES, node, key, null, null);

				// Read and parse response.
				final CAERCommunication.caerControlResponse typeResponse = CAERCommunication.readResponse(controlSocket,
					netBuf);

				if ((typeResponse.getAction() != CAERCommunication.caerControlConfigAction.ERROR)
					&& (typeResponse.getType() == SSHSType.STRING)) {
					// For each key and type, we get the current value and then
					// build the configuration control knob.
					for (final String type : typeResponse.getMessage().split("\0")) {
						// Query current value.
						CAERCommunication.sendCommand(controlSocket, netBuf,
							CAERCommunication.caerControlConfigAction.GET, node, key, SSHSType.getTypeByName(type),
							null);

						// Read and parse response.
						final CAERCommunication.caerControlResponse valueResponse = CAERCommunication
							.readResponse(controlSocket, netBuf);

						if ((valueResponse.getAction() != CAERCommunication.caerControlConfigAction.ERROR)
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
		CAERCommunication.sendCommand(controlSocket, netBuf, CAERCommunication.caerControlConfigAction.GET_CHILDREN,
			node, null, null, null);

		// Read and parse response.
		final CAERCommunication.caerControlResponse childResponse = CAERCommunication.readResponse(controlSocket,
			netBuf);

		if ((childResponse.getAction() != CAERCommunication.caerControlConfigAction.ERROR)
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

		final Label l = GUISupport.addLabel(configBox, String.format("%s (%s):", key, type), null);

		l.setPrefWidth(200);
		l.setAlignment(Pos.CENTER_RIGHT);

		switch (SSHSType.getTypeByName(type)) {
			case BOOL:
				GUISupport.addCheckBox(configBox, null, value.equals("true")).selectedProperty()
					.addListener(new ChangeListener<Boolean>() {
						@SuppressWarnings("unused")
						@Override
						public void changed(final ObservableValue<? extends Boolean> observable, final Boolean oldValue,
							final Boolean newValue) {
							// Send new value to cAER control server.
							CAERCommunication.sendCommand(controlSocket, netBuf,
								CAERCommunication.caerControlConfigAction.PUT, node, key, null,
								new SSHSAttributeValue(newValue));
							CAERCommunication.readResponse(controlSocket, netBuf);
						}
					});
				break;

			case BYTE: {
				final IntegerProperty backendValue = GUISupport.addTextNumberFieldWithSlider(configBox,
					Byte.parseByte(value), 0, Byte.MAX_VALUE);
				backendValue.addListener(new ChangeListener<Number>() {
					@SuppressWarnings("unused")
					@Override
					public void changed(final ObservableValue<? extends Number> observable, final Number oldValue,
						final Number newValue) {
						// Send new value to cAER control server.
						CAERCommunication.sendCommand(controlSocket, netBuf,
							CAERCommunication.caerControlConfigAction.PUT, node, key, null,
							new SSHSAttributeValue(newValue.byteValue()));
						CAERCommunication.readResponse(controlSocket, netBuf);
					}
				});
				break;
			}

			case SHORT: {
				final IntegerProperty backendValue = GUISupport.addTextNumberFieldWithSlider(configBox,
					Short.parseShort(value), 0, Short.MAX_VALUE);
				backendValue.addListener(new ChangeListener<Number>() {
					@SuppressWarnings("unused")
					@Override
					public void changed(final ObservableValue<? extends Number> observable, final Number oldValue,
						final Number newValue) {
						// Send new value to cAER control server.
						CAERCommunication.sendCommand(controlSocket, netBuf,
							CAERCommunication.caerControlConfigAction.PUT, node, key, null,
							new SSHSAttributeValue(newValue.shortValue()));
						CAERCommunication.readResponse(controlSocket, netBuf);
					}
				});
				break;
			}

			case INT: {
				final IntegerProperty backendValue = GUISupport.addTextNumberFieldWithSlider(configBox,
					Integer.parseInt(value), 0, Integer.MAX_VALUE);
				backendValue.addListener(new ChangeListener<Number>() {
					@SuppressWarnings("unused")
					@Override
					public void changed(final ObservableValue<? extends Number> observable, final Number oldValue,
						final Number newValue) {
						// Send new value to cAER control server.
						CAERCommunication.sendCommand(controlSocket, netBuf,
							CAERCommunication.caerControlConfigAction.PUT, node, key, null,
							new SSHSAttributeValue(newValue.intValue()));
						CAERCommunication.readResponse(controlSocket, netBuf);
					}
				});
				break;
			}

			case LONG: {
				final LongProperty backendValue = GUISupport.addTextNumberFieldWithSlider(configBox,
					Long.parseLong(value), 0, Long.MAX_VALUE);
				backendValue.addListener(new ChangeListener<Number>() {
					@SuppressWarnings("unused")
					@Override
					public void changed(final ObservableValue<? extends Number> observable, final Number oldValue,
						final Number newValue) {
						// Send new value to cAER control server.
						CAERCommunication.sendCommand(controlSocket, netBuf,
							CAERCommunication.caerControlConfigAction.PUT, node, key, null,
							new SSHSAttributeValue(newValue.longValue()));
						CAERCommunication.readResponse(controlSocket, netBuf);
					}
				});
				break;
			}

			case FLOAT: {
				final DoubleProperty backendValue = GUISupport.addTextNumberFieldWithSlider(configBox,
					Float.parseFloat(value), Float.MIN_VALUE, Float.MAX_VALUE);
				backendValue.addListener(new ChangeListener<Number>() {
					@SuppressWarnings("unused")
					@Override
					public void changed(final ObservableValue<? extends Number> observable, final Number oldValue,
						final Number newValue) {
						// Send new value to cAER control server.
						CAERCommunication.sendCommand(controlSocket, netBuf,
							CAERCommunication.caerControlConfigAction.PUT, node, key, null,
							new SSHSAttributeValue(newValue.floatValue()));
						CAERCommunication.readResponse(controlSocket, netBuf);
					}
				});
				break;
			}

			case DOUBLE: {
				final DoubleProperty backendValue = GUISupport.addTextNumberFieldWithSlider(configBox,
					Double.parseDouble(value), Double.MIN_VALUE, Double.MAX_VALUE);
				backendValue.addListener(new ChangeListener<Number>() {
					@SuppressWarnings("unused")
					@Override
					public void changed(final ObservableValue<? extends Number> observable, final Number oldValue,
						final Number newValue) {
						// Send new value to cAER control server.
						CAERCommunication.sendCommand(controlSocket, netBuf,
							CAERCommunication.caerControlConfigAction.PUT, node, key, null,
							new SSHSAttributeValue(newValue.doubleValue()));
						CAERCommunication.readResponse(controlSocket, netBuf);
					}
				});
				break;
			}

			case STRING:
				GUISupport.addTextField(configBox, value).textProperty().addListener(new ChangeListener<String>() {
					@SuppressWarnings("unused")
					@Override
					public void changed(final ObservableValue<? extends String> observable, final String oldValue,
						final String newValue) {
						// Send new value to cAER control server.
						CAERCommunication.sendCommand(controlSocket, netBuf,
							CAERCommunication.caerControlConfigAction.PUT, node, key, null,
							new SSHSAttributeValue(newValue));
						CAERCommunication.readResponse(controlSocket, netBuf);
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

	private void cleanupConnection() throws IOException {
		mainGUI.getChildren().clear();

		if (controlSocket != null) {
			controlSocket.close();
			controlSocket = null;
		}
	}
}
