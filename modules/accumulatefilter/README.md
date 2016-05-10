# Accumulate Filter

The filter aggregates polarity (x,y) events by time, to a 1D semantic representation, 
or to a 2D slice. 

`bool caerAccomulateFilter(PolarityEventPacket input, int dT, PolarityEventPacket dvsBuffer, vector<unsigned int> 1dBuffer, matrix<uint, uint> 2dBuffer, enum polarityMode = ignore)`

Where:
 * `input` is the packetFilter passed around in the mainloop
 * `dT` time (ms); <0 => do not aggregate by time, directly pass; >0 => collect events for `dT` ms, write them to `dvsBuffer` after this period. 
 * `dvsBuffer` event packet released only after `dT` passed. `input` packets received within this period are accumulated to this `dvsBuffer` and function returns `false`. When time of events in in the packet exceeds the threshold (= time of the earliest event in internal buffer + dT), internal buffer is dumped to dvsBuffer, start time is reset, and the function returns `true`. Set `dvsBuffer` to `null` to disable the DVS event aggregation by time. 
 * `1dBuffer` is a 1D vector that contains accumulated "1D representation of the 2D events/points"; the transformation is semantic, in the meaning it tries to keep distance: `eucleidian distance ||(x1,y1), (x2,y2)|| ~ ||vector1, vector2||`.
 * `2dBuffer` is a 2D matrix representing the retina(camera) resolution. A new point is simply written to matrix(x,y).
 * `polarityMode` is an `enum {POLARITY_ON, POLARITY_OFF, POLARITY_REPLACE, POLARITY_IGNORED}`, defaulting to the ignored. 
  * `POLARITY_ON`: only ON events (polarity=1) are considered
  * `POLARITY_OFF`: only OFF events considered
  * `POlARITY_REPLACE`: an ON event sets the respective bit 1, OFF event to 0, a bit can be rewritten by a newer event
  * `POLARITY_IGNORED`: both ON/OFF events are threated the same and set respective bit ON.

