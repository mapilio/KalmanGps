# KalmanGps

## Overview

KalmanGps is an advanced GPS correction tool that utilizes sensor data, including magnetometer, gyroscope, accelerometer, and GPS information, to provide accurate and corrected latitude and longitude coordinates. This application is particularly useful for scenarios where precise location data is essential, such as navigation systems, robotics, or any application requiring accurate positioning.

## Features

- **Sensor Fusion:** KalmanGps integrates data from multiple sensors, including magnetometer, gyroscope, and accelerometer, to enhance the accuracy of GPS coordinates.
  
- **Real-time Correction:** Corrects GPS coordinates in real-time, providing continuously refined location information.

- **Timestamp Precision:** Utilizes high-resolution timestamps (`timestamp_ns`) for precise time synchronization.

- **Adaptive Correction:** The Kalman filter adapts to changing conditions, ensuring reliable correction under various scenarios.

- **User-Configurable:** Offers flexibility with configurable settings to adjust the correction algorithm based on specific requirements.

- **Export Corrected Data:** Easily export the corrected latitude and longitude data for further analysis or integration into other applications.


## Getting Started

These instructions will help you get a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

Make sure you have the following installed before proceeding:

- [Python](https://www.python.org/) (version 3.x)
- [Pip](https://pip.pypa.io/en/stable/installation/)
- [Virtualenv](https://virtualenv.pypa.io/) (optional but recommended)

### Installation
```bash
pip install kalmangps
```
or clone the repository and install the necessary dependencies:
```bash
# Clone the repository
git clone https://github.com/your-username/KalmanGps.git

# Change into the project directory
cd KalmanGps

# Create a virtual environment (optional but recommended)
virtualenv venv

# Activate the virtual environment
source venv/bin/activate  # On Windows: .\venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```
### Usage

1. **Input:** Path to the input data file containing sensor information (magnetometer, gyro, ACC, timestamp_ns, and GPS data).
2. **Output:**  Path to save the corrected latitude and longitude data.
3. **mapit:** Flag to visualize the corrected data on a map. If specified, the application will generate a map displaying the corrected GPS coordinates.

Example command:

```bash
python kalman_processor.py --input_path /path/to/input_csv --output_path /path/to/output.csv --mapit 
```
or

```python
from kalmangps.kalman_processor import Kalman
input_path=r'' 
output_path=r''
klmn = Kalman(csv_path=input_path, output_path=output_path, mapit=True)
output = klmn()
```


### Contribution

Contributions and feedback are welcome! If you encounter any issues or have suggestions for improvements, please open an issue or submit a pull request.

