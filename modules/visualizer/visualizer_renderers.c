#include "visualizer.h"
#include "base/mainloop.h"
#include "modules/ini/dynapse_common.h"

#include <math.h>
#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_ttf.h>

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>
#include <libcaer/events/imu6.h>
#include <libcaer/events/point2d.h>
#include <libcaer/events/spike.h>

bool caerVisualizerRendererPolarityEvents(caerVisualizerPublicState state, caerEventPacketContainer container,
bool doClear) {
	UNUSED_ARGUMENT(state);

	// Clear bitmap to black to erase old events.
	if (doClear) {
		al_clear_to_color(al_map_rgb(0, 0, 0));
	}

	caerEventPacketHeader polarityEventPacketHeader = caerEventPacketContainerFindEventPacketByType(container,
		POLARITY_EVENT);

	if (polarityEventPacketHeader == NULL || caerEventPacketHeaderGetEventValid(polarityEventPacketHeader) == 0) {
		return (false);
	}

	// Render all valid events.
	CAER_POLARITY_ITERATOR_VALID_START((caerPolarityEventPacket) polarityEventPacketHeader)
		if (caerPolarityEventGetPolarity(caerPolarityIteratorElement)) {
			// ON polarity (green).
			al_put_pixel(caerPolarityEventGetX(caerPolarityIteratorElement),
				caerPolarityEventGetY(caerPolarityIteratorElement), al_map_rgb(0, 255, 0));
		}
		else {
			// OFF polarity (red).
			al_put_pixel(caerPolarityEventGetX(caerPolarityIteratorElement),
				caerPolarityEventGetY(caerPolarityIteratorElement), al_map_rgb(255, 0, 0));
		}CAER_POLARITY_ITERATOR_VALID_END

	return (true);
}

bool caerVisualizerRendererFrameEvents(caerVisualizerPublicState state, caerEventPacketContainer container,
bool doClear) {
	UNUSED_ARGUMENT(state);
	UNUSED_ARGUMENT(doClear); // Don't erase last frame.

	caerEventPacketHeader frameEventPacketHeader = caerEventPacketContainerFindEventPacketByType(container,
		FRAME_EVENT);

	if (frameEventPacketHeader == NULL || caerEventPacketHeaderGetEventValid(frameEventPacketHeader) == 0) {
		return (false);
	}

	// Render only the last, valid frame.
	caerFrameEventPacket currFramePacket = (caerFrameEventPacket) frameEventPacketHeader;
	caerFrameEvent currFrameEvent;

	for (int32_t i = caerEventPacketHeaderGetEventNumber(&currFramePacket->packetHeader) - 1; i >= 0; i--) {
		currFrameEvent = caerFrameEventPacketGetEvent(currFramePacket, i);

		// Only operate on the last, valid frame.
		if (caerFrameEventIsValid(currFrameEvent)) {
			// Always clear bitmap to black to erase old frame, this is needed in case ROI
			// has its position moving around in the screen.
			al_clear_to_color(al_map_rgb(0, 0, 0));

			// Copy the frame content to the render bitmap.
			// Use frame sizes to correctly support small ROI frames.
			int32_t frameSizeX = caerFrameEventGetLengthX(currFrameEvent);
			int32_t frameSizeY = caerFrameEventGetLengthY(currFrameEvent);
			int32_t framePositionX = caerFrameEventGetPositionX(currFrameEvent);
			int32_t framePositionY = caerFrameEventGetPositionY(currFrameEvent);
			enum caer_frame_event_color_channels frameChannels = caerFrameEventGetChannelNumber(currFrameEvent);

			for (int32_t y = 0; y < frameSizeY; y++) {
				for (int32_t x = 0; x < frameSizeX; x++) {
					ALLEGRO_COLOR color;

					switch (frameChannels) {
						case GRAYSCALE: {
							uint8_t pixel = U8T(caerFrameEventGetPixelUnsafe(currFrameEvent, x, y) >> 8);
							color = al_map_rgb(pixel, pixel, pixel);
							break;
						}

						case RGB: {
							uint8_t pixelR = U8T(caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 0) >> 8);
							uint8_t pixelG = U8T(caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 1) >> 8);
							uint8_t pixelB = U8T(caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 2) >> 8);
							color = al_map_rgb(pixelR, pixelG, pixelB);
							break;
						}

						case RGBA:
						default: {
							uint8_t pixelR = U8T(caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 0) >> 8);
							uint8_t pixelG = U8T(caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 1) >> 8);
							uint8_t pixelB = U8T(caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 2) >> 8);
							uint8_t pixelA = U8T(caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 3) >> 8);
							color = al_map_rgba(pixelR, pixelG, pixelB, pixelA);
							break;
						}
					}

					al_put_pixel((framePositionX + x), (framePositionY + y), color);
				}
			}

			return (true);
		}
	}

	return (false);
}

#define RESET_LIMIT_POS(VAL, LIMIT) if ((VAL) > (LIMIT)) { (VAL) = (LIMIT); }
#define RESET_LIMIT_NEG(VAL, LIMIT) if ((VAL) < (LIMIT)) { (VAL) = (LIMIT); }

bool caerVisualizerRendererIMU6Events(caerVisualizerPublicState state, caerEventPacketContainer container, bool doClear) {
	// Clear bitmap to black to erase old events.
	if (doClear) {
		al_clear_to_color(al_map_rgb(0, 0, 0));
	}

	caerEventPacketHeader imu6EventPacketHeader = caerEventPacketContainerFindEventPacketByType(container, IMU6_EVENT);

	if (imu6EventPacketHeader == NULL || caerEventPacketHeaderGetEventValid(imu6EventPacketHeader) == 0) {
		return (false);
	}

	float scaleFactorAccel = 30;
	float scaleFactorGyro = 15;
	float lineThickness = 4;
	float maxSizeX = (float) state->bitmapRendererSizeX;
	float maxSizeY = (float) state->bitmapRendererSizeY;

	ALLEGRO_COLOR accelColor = al_map_rgb(0, 255, 0);
	ALLEGRO_COLOR gyroColor = al_map_rgb(255, 0, 255);

	float centerPointX = maxSizeX / 2;
	float centerPointY = maxSizeY / 2;

	float accelX = 0, accelY = 0, accelZ = 0;
	float gyroX = 0, gyroY = 0, gyroZ = 0;

	// Iterate over valid IMU events and average them.
	// This somewhat smoothes out the rendering.
	CAER_IMU6_ITERATOR_VALID_START((caerIMU6EventPacket) imu6EventPacketHeader)
		accelX += caerIMU6EventGetAccelX(caerIMU6IteratorElement);
		accelY += caerIMU6EventGetAccelY(caerIMU6IteratorElement);
		accelZ += caerIMU6EventGetAccelZ(caerIMU6IteratorElement);

		gyroX += caerIMU6EventGetGyroX(caerIMU6IteratorElement);
		gyroY += caerIMU6EventGetGyroY(caerIMU6IteratorElement);
		gyroZ += caerIMU6EventGetGyroZ(caerIMU6IteratorElement);
	CAER_IMU6_ITERATOR_VALID_END

	// Normalize values.
	int32_t validEvents = caerEventPacketHeaderGetEventValid(imu6EventPacketHeader);

	accelX /= (float) validEvents;
	accelY /= (float) validEvents;
	accelZ /= (float) validEvents;

	gyroX /= (float) validEvents;
	gyroY /= (float) validEvents;
	gyroZ /= (float) validEvents;

	// Acceleration X, Y as lines. Z as a circle.
	float accelXScaled = centerPointX - accelX * scaleFactorAccel;
	RESET_LIMIT_POS(accelXScaled, maxSizeX - 2 - lineThickness);
	RESET_LIMIT_NEG(accelXScaled, 1 + lineThickness);
	float accelYScaled = centerPointY - accelY * scaleFactorAccel;
	RESET_LIMIT_POS(accelYScaled, maxSizeY - 2 - lineThickness);
	RESET_LIMIT_NEG(accelYScaled, 1 + lineThickness);
	float accelZScaled = fabsf(accelZ * scaleFactorAccel);
	RESET_LIMIT_POS(accelZScaled, centerPointY - 2 - lineThickness); // Circle max.
	RESET_LIMIT_NEG(accelZScaled, 1); // Circle min.

	al_draw_line(centerPointX, centerPointY, accelXScaled, accelYScaled, accelColor, lineThickness);
	al_draw_circle(centerPointX, centerPointY, accelZScaled, accelColor, lineThickness);

	// TODO: Add text for values. Check that displayFont is not NULL.
	//char valStr[128];
	//snprintf(valStr, 128, "%.2f,%.2f g", (double) accelX, (double) accelY);
	//al_draw_text(state->displayFont, accelColor, accelXScaled, accelYScaled, 0, valStr);

	// Gyroscope pitch(X), yaw(Y), roll(Z) as lines.
	float gyroXScaled = centerPointY + gyroX * scaleFactorGyro;
	RESET_LIMIT_POS(gyroXScaled, maxSizeY - 2 - lineThickness);
	RESET_LIMIT_NEG(gyroXScaled, 1 + lineThickness);
	float gyroYScaled = centerPointX + gyroY * scaleFactorGyro;
	RESET_LIMIT_POS(gyroYScaled, maxSizeX - 2 - lineThickness);
	RESET_LIMIT_NEG(gyroYScaled, 1 + lineThickness);
	float gyroZScaled = centerPointX - gyroZ * scaleFactorGyro;
	RESET_LIMIT_POS(gyroZScaled, maxSizeX - 2 - lineThickness);
	RESET_LIMIT_NEG(gyroZScaled, 1 + lineThickness);

	al_draw_line(centerPointX, centerPointY, gyroYScaled, gyroXScaled, gyroColor, lineThickness);
	al_draw_line(centerPointX, centerPointY - 20, gyroZScaled, centerPointY - 20, gyroColor, lineThickness);

	return (true);
}

bool caerVisualizerRendererPoint2DEvents(caerVisualizerPublicState state, caerEventPacketContainer container,
bool doClear) {
	UNUSED_ARGUMENT(state);

	// Clear bitmap to black to erase old events.
	if (doClear) {
		al_clear_to_color(al_map_rgb(0, 0, 0));
	}

	caerEventPacketHeader point2DEventPacketHeader = caerEventPacketContainerFindEventPacketByType(container,
		POINT2D_EVENT);

	if (point2DEventPacketHeader == NULL || caerEventPacketHeaderGetEventValid(point2DEventPacketHeader) == 0) {
		return (false);
	}

	// Render all valid events.
	CAER_POINT2D_ITERATOR_VALID_START((caerPoint2DEventPacket) point2DEventPacketHeader)
		float x = caerPoint2DEventGetX(caerPoint2DIteratorElement);
		float y = caerPoint2DEventGetY(caerPoint2DIteratorElement);

		// Display points in blue.
		al_put_pixel((int) x, (int) y, al_map_rgb(0, 255, 255));
	CAER_POINT2D_ITERATOR_VALID_END

	return (true);
}

bool caerVisualizerRendererSpikeEventsRaster(caerVisualizerPublicState state, caerEventPacketContainer container,
bool doClear) {
	UNUSED_ARGUMENT(state);

	// Clear bitmap to black to erase old events.
	if (doClear) {
		al_clear_to_color(al_map_rgb(0, 0, 0));
	}

	caerEventPacketHeader spikeEventPacketHeader = caerEventPacketContainerFindEventPacketByType(container,
		SPIKE_EVENT);

	if (spikeEventPacketHeader == NULL || caerEventPacketHeaderGetEventValid(spikeEventPacketHeader) == 0) {
		return (false);
	}

	// get size bitmap's size
	int32_t sizeX = state->bitmapRendererSizeX;
	int32_t sizeY = state->bitmapRendererSizeY;

	// find max and min TS
	int32_t min_ts = INT_MAX;
	int32_t max_ts = INT_MIN;
	CAER_SPIKE_ITERATOR_ALL_START( (caerSpikeEventPacket) spikeEventPacketHeader )
		int32_t ts = caerSpikeEventGetTimestamp(caerSpikeIteratorElement);
		if (ts > max_ts) {
			max_ts = ts;
		}
		if (ts < min_ts) {
			min_ts = ts;
		}CAER_SPIKE_ITERATOR_ALL_END
	// time span
	int32_t time_span = max_ts - min_ts;
	float scalex = 0.0;
	if (time_span > 0) {
		scalex = ((float) sizeX / 2) / ((float) time_span); // two rasterplots in x
	}
	float scaley = ((float) sizeY / 2) / ((float) DYNAPSE_CONFIG_NUMNEURONS); // two rasterplots in y
	int32_t new_x = 0;

	// Render all spikes.
	CAER_SPIKE_ITERATOR_ALL_START( (caerSpikeEventPacket) spikeEventPacketHeader )

	// get core id
		int8_t coreId = caerSpikeEventGetSourceCoreID(caerSpikeIteratorElement);
		int32_t ts = caerSpikeEventGetTimestamp(caerSpikeIteratorElement);
		ts = ts - min_ts;
		new_x = (int32_t) floor((float) ts * scalex);

		// get x,y position
		uint16_t y = caerSpikeEventGetY(caerSpikeIteratorElement);
		uint16_t x = caerSpikeEventGetX(caerSpikeIteratorElement);

		// calculate coordinates in the screen
		// select chip
		uint8_t chipId = caerSpikeEventGetChipID(caerSpikeIteratorElement);
		// adjust coordinate for chip
		if (chipId == DYNAPSE_CONFIG_DYNAPSE_U3) {
			x = x - DYNAPSE_CONFIG_XCHIPSIZE;
			y = y - DYNAPSE_CONFIG_YCHIPSIZE;
		}
		else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U2) {
			y = y - DYNAPSE_CONFIG_YCHIPSIZE;
		}
		else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U1) {
			x = x - DYNAPSE_CONFIG_XCHIPSIZE;
		}
		else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U0 + 1) { // sram can't be zero
			;
		}
		// adjust coordinates for cores
		if (coreId == 1) {
			y = y - DYNAPSE_CONFIG_NEUCOL;
		}
		else if (coreId == 0) {
			;
		}
		else if (coreId == 2) {
			x = x - DYNAPSE_CONFIG_NEUCOL;
		}
		else if (coreId == 3) {
			x = x - DYNAPSE_CONFIG_NEUCOL;
			y = y - DYNAPSE_CONFIG_NEUCOL;
		}

		uint32_t indexLin = x * DYNAPSE_CONFIG_NEUCOL + y;
		uint32_t new_y = indexLin + (coreId * DYNAPSE_CONFIG_NUMNEURONS_CORE);

		// adjust coordinate for plot
		new_y = (int32_t) floor((float) new_y * scaley);
		// move
		if (chipId == DYNAPSE_CONFIG_DYNAPSE_U3) {
			new_x = (int32_t) floor((float) new_x + ((float) sizeX / 2));
			new_y = (uint32_t) floor((float) new_y + (float) sizeY / 2.0);
		}
		else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U2) {
			new_x = (int32_t) floor((float) new_x + ((float) sizeX / 2));
		}
		else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U1) {
			new_y = (uint32_t) floor((float) new_y + (float) sizeY / 2.0);
		}
		else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U0 + 1) { // sram can't be zero
			;
		}
		// draw borders
		for (int x = 0; x < sizeX; x++) {
			al_put_pixel(x, sizeY / 2, al_map_rgb(255, 255, 255));
		}
		for (int y = 0; y < sizeY; y++) {
			al_put_pixel(sizeX / 2, y, al_map_rgb(255, 255, 255));
		}

		// draw pixels (neurons might be merged due to aliasing..)
		al_put_pixel(new_x, new_y, al_map_rgb(coreId * 0, 255, 0 * coreId));

		//caerLog(CAER_LOG_NOTICE, __func__, "chipId %d, new_x %d, new_y %d, indexLin %d\n", chipId, new_x, new_y, indexLin);

	CAER_SPIKE_ITERATOR_ALL_END

	return (true);
}

bool caerVisualizerRendererSpikeEvents(caerVisualizerPublicState state, caerEventPacketContainer container,
bool doClear) {
	UNUSED_ARGUMENT(state);

	// Clear bitmap to black to erase old events.
	if (doClear) {
		al_clear_to_color(al_map_rgb(0, 0, 0));
	}

	caerEventPacketHeader spikeEventPacketHeader = caerEventPacketContainerFindEventPacketByType(container,
		SPIKE_EVENT);

	if (spikeEventPacketHeader == NULL || caerEventPacketHeaderGetEventValid(spikeEventPacketHeader) == 0) {
		return (false);
	}

	// Render all spikes.
	CAER_SPIKE_ITERATOR_ALL_START( (caerSpikeEventPacket) spikeEventPacketHeader )

		// get core id
		uint8_t coreId = caerSpikeEventGetSourceCoreID(caerSpikeIteratorElement);
		uint8_t chipId = caerSpikeEventGetChipID(caerSpikeIteratorElement);
		//get x,y position
		uint16_t y = caerSpikeEventGetY(caerSpikeIteratorElement);
		uint16_t x = caerSpikeEventGetX(caerSpikeIteratorElement);

		if (coreId == 0){
			al_put_pixel(x, y, al_map_rgb( 0, 255, 0 ));
			}
		else if (coreId == 1) {
			al_put_pixel(x, y, al_map_rgb( 0, 0, 255 ));
			}
		else if(coreId == 2) {
			al_put_pixel(x, y, al_map_rgb( 255, 0, 0 ));
			}
		else if(coreId == 3) {
			al_put_pixel(x, y, al_map_rgb( 255, 255, 0));
			}

	CAER_SPIKE_ITERATOR_ALL_END

	return (true);
}

bool caerVisualizerMultiRendererPolarityAndFrameEvents(caerVisualizerPublicState state,
	caerEventPacketContainer container, bool doClear) {
	UNUSED_ARGUMENT(doClear); // Don't clear old frames, add events on top.

	bool drewFrameEvents = caerVisualizerRendererFrameEvents(state, container, false);

	bool drewPolarityEvents = caerVisualizerRendererPolarityEvents(state, container, false);

	return (drewFrameEvents || drewPolarityEvents);
}
