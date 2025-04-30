## Challenges Faced and Solutions Implemented

### • Power Supply Reliability
When all the sensors in the system were powered by the same power supply along with the ejection system, the voltage dropped during high current demand from the ejection system, causing the supply to become inadequate for the On-Board Computer (OBC) to operate.  
**Solution:** We resolved the issue by providing dedicated power supplies to both the OBC and the ejection system.

### • Sensor Calibration Drift
We noticed that environmental conditions could cause sensors like the BMP180 to drift. For example, when the sensor was tested under direct sunlight or at night, the ejection system got triggered unexpectedly. The altitude data from the sensor showed a negative value, which falsely triggered ignition.  
**Solution:** We used black tape to cover the BMP180 sensor during testing under sunlight to reduce environmental interference.

### • Prolonged Initialization of GPS Module
The GPS module (NEO-6M) experienced significant delays in obtaining a GPS fix, particularly during initial setup. These delays were most noticeable during cold start conditions, where the GPS needed to download satellite data from scratch.  
**Solution:** The system was pre-tested in open environments to ensure faster satellite lock during actual deployment.

### • LoRa Communication Issues
During testing, the LoRa module (RA-02) occasionally failed to transmit all data packets reliably. This resulted in incomplete data reception at the ground station. Possible causes included environmental interference, range limitations, bandwidth constraints, or transmission delays.  
**Solution:** We recommend switching to more reliable communication modules than the LoRa RA-02, while still maintaining affordability.
