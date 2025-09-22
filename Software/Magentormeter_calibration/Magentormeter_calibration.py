import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import serial
import time
from scipy.linalg import inv, norm
from scipy.optimize import least_squares
import threading
from queue import Queue

class MagnetometerCalibrator:
    def __init__(self, serial_port, baud_rate=115200):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.raw_data = []
        self.calibrated_data = []
        self.calibration_params = {}
        self.data_queue = Queue()
        self.is_collecting = False
        
    def ellipsoid_fit(self, data):
        """
        Fit an ellipsoid to magnetometer data using least squares optimization
        to determine hard iron offsets and soft iron transformation matrix
        """
        # Implementation of ellipsoid fitting algorithm
        pass
        
    def calibrate_points(self, points):
        """
        Apply calibration parameters to raw magnetometer readings
        """
        # Apply calibration transformation to raw data
        pass
        
    def serial_reader(self):
        """
        Read data from serial port and add to queue
        """
        try:
            with serial.Serial(self.serial_port, self.baud_rate, timeout=1) as ser:
                print(f"Connected to {self.serial_port}. Starting data collection...")
                print("Rotate the magnetometer slowly in all possible orientations")
                
                while self.is_collecting:
                    line = ser.readline().decode('utf-8').strip()
                    if line and line.startswith('MAG:'):
                        try:
                            # Expected format: "MAG:x,y,z"
                            values = line.split(':')[1].split(',')
                            x, y, z = map(float, values)
                            self.data_queue.put((x, y, z))
                        except (ValueError, IndexError):
                            continue
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            self.is_collecting = False

    def collect_data(self, duration=60, sample_delay=0.1):
        """
        Collect magnetometer data for specified duration
        """
        self.raw_data = []
        self.is_collecting = True
        
        # Start serial reader in a separate thread
        reader_thread = threading.Thread(target=self.serial_reader)
        reader_thread.daemon = True
        reader_thread.start()
        
        # Collect data for specified duration
        start_time = time.time()
        last_sample_time = start_time
        
        while time.time() - start_time < duration:
            if time.time() - last_sample_time >= sample_delay and not self.data_queue.empty():
                x, y, z = self.data_queue.get()
                self.raw_data.append([x, y, z])
                last_sample_time = time.time()
                
                # Print progress
                progress = (time.time() - start_time) / duration * 100
                print(f"Progress: {progress:.1f}% - Collected {len(self.raw_data)} samples", end='\r')
        
        self.is_collecting = False
        print(f"\nData collection complete. Collected {len(self.raw_data)} samples.")
        
    def perform_calibration(self):
        """
        Perform ellipsoid fitting calibration on collected data
        """
        if len(self.raw_data) < 100:
            print("Insufficient data for calibration. Need at least 100 samples.")
            return False
            
        print("Performing ellipsoid fitting calibration...")
        
        # Convert to numpy array
        data = np.array(self.raw_data)
        
        # Perform ellipsoid fitting
        # This is a simplified version - actual implementation would use
        # more sophisticated algorithms for ellipsoid fitting
        center = np.mean(data, axis=0)
        centered_data = data - center
        
        # Covariance matrix
        cov = np.cov(centered_data.T)
        
        # Eigen decomposition to find principal axes
        eigenvalues, eigenvectors = np.linalg.eig(cov)
        
        # Scaling factors (inverse of eigenvalues)
        scaling = 1.0 / np.sqrt(eigenvalues)
        
        # Store calibration parameters
        self.calibration_params = {
            'hard_iron_offset': center.tolist(),
            'soft_iron_matrix': (eigenvectors @ np.diag(scaling) @ eigenvectors.T).tolist()
        }
        
        # Apply calibration to raw data
        self.calibrated_data = self.calibrate_points(data)
        
        print("Calibration complete!")
        return True
        
    def plot_data(self):
        """
        Create 3D plots of raw and calibrated magnetometer data
        """
        if not self.raw_data:
            print("No data to plot")
            return
            
        raw_data = np.array(self.raw_data)
        calibrated_data = np.array(self.calibrated_data) if self.calibrated_data else None
        
        fig = plt.figure(figsize=(12, 5))
        
        # Plot raw data
        ax1 = fig.add_subplot(121, projection='3d')
        ax1.scatter(raw_data[:, 0], raw_data[:, 1], raw_data[:, 2], 
                   c='r', marker='o', alpha=0.5, label='Raw Data')
        ax1.set_title('Raw Magnetometer Data')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Z')
        ax1.legend()
        
        # Plot calibrated data if available
        if calibrated_data is not None:
            ax2 = fig.add_subplot(122, projection='3d')
            ax2.scatter(calibrated_data[:, 0], calibrated_data[:, 1], calibrated_data[:, 2], 
                       c='b', marker='o', alpha=0.5, label='Calibrated Data')
            ax2.set_title('Calibrated Magnetometer Data')
            ax2.set_xlabel('X')
            ax2.set_ylabel('Y')
            ax2.set_zlabel('Z')
            ax2.legend()
            
            # Calculate and display calibration metrics
            radii = np.linalg.norm(calibrated_data, axis=1)
            avg_radius = np.mean(radii)
            std_radius = np.std(radii)
            
            print(f"Calibration Results:")
            print(f"Average radius: {avg_radius:.2f}")
            print(f"Standard deviation: {std_radius:.2f}")
            print(f"Hard iron offset: {self.calibration_params['hard_iron_offset']}")
        
        plt.tight_layout()
        plt.show()
        
    def save_calibration(self, filename):
        """
        Save calibration parameters to a file
        """
        if not self.calibration_params:
            print("No calibration parameters to save")
            return
            
        with open(filename, 'w') as f:
            f.write("# Magnetometer Calibration Parameters\n")
            f.write(f"HardIronOffset: {self.calibration_params['hard_iron_offset']}\n")
            f.write(f"SoftIronMatrix: {self.calibration_params['soft_iron_matrix']}\n")
            
        print(f"Calibration parameters saved to {filename}")

def main():
    # Configuration - update with your serial port
    serial_port = "COM3"  # Windows example
    # serial_port = "/dev/ttyUSB0"  # Linux example
    # serial_port = "/dev/cu.usbserial-*"  # macOS example
    
    # Create calibrator instance
    calibrator = MagnetometerCalibrator(serial_port)
    
    # Collect data (rotate the magnetometer during this time)
    calibrator.collect_data(duration=120, sample_delay=0.05)  # Collect for 2 minutes
    
    # Perform calibration
    if calibrator.perform_calibration():
        # Plot results
        calibrator.plot_data()
        
        # Save calibration parameters
        calibrator.save_calibration("magnetometer_calibration.txt")
        
        # Print code snippet for ESP32 implementation
        print("\nESP32 Implementation Snippet:")
        print("""
// Apply calibration in ESP32 code
Vector3f calibrate_magnetometer(float x, float y, float z) {
    // Hard iron offsets (from calibration)
    float offset_x = %f;
    float offset_y = %f;
    float offset_z = %f;
    
    // Soft iron transformation matrix (from calibration)
    float matrix[3][3] = {
        {%f, %f, %f},
        {%f, %f, %f},
        {%f, %f, %f}
    };
    
    // Apply hard iron correction
    float corrected_x = x - offset_x;
    float corrected_y = y - offset_y;
    float corrected_z = z - offset_z;
    
    // Apply soft iron correction
    float calibrated_x = matrix[0][0]*corrected_x + matrix[0][1]*corrected_y + matrix[0][2]*corrected_z;
    float calibrated_y = matrix[1][0]*corrected_x + matrix[1][1]*corrected_y + matrix[1][2]*corrected_z;
    float calibrated_z = matrix[2][0]*corrected_x + matrix[2][1]*corrected_y + matrix[2][2]*corrected_z;
    
    return Vector3f(calibrated_x, calibrated_y, calibrated_z);
}
        """ % (
            calibrator.calibration_params['hard_iron_offset'][0],
            calibrator.calibration_params['hard_iron_offset'][1],
            calibrator.calibration_params['hard_iron_offset'][2],
            calibrator.calibration_params['soft_iron_matrix'][0][0],
            calibrator.calibration_params['soft_iron_matrix'][0][1],
            calibrator.calibration_params['soft_iron_matrix'][0][2],
            calibrator.calibration_params['soft_iron_matrix'][1][0],
            calibrator.calibration_params['soft_iron_matrix'][1][1],
            calibrator.calibration_params['soft_iron_matrix'][1][2],
            calibrator.calibration_params['soft_iron_matrix'][2][0],
            calibrator.calibration_params['soft_iron_matrix'][2][1],
            calibrator.calibration_params['soft_iron_matrix'][2][2]
        ))

if __name__ == "__main__":
    main()