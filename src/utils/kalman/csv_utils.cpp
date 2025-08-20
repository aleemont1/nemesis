#include "csv_utils.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

// Get Data from files
// Acceleration
std::vector<float> getAcc(int lineNumberSought) {
    std::string line, csvItem;
    std::vector<float> acc;
    std::ifstream myfile("../csv_measurements/Sensor Simulation/Sensors/acc_sensor.csv");

    int lineNumber = 1;
    if (myfile.is_open()) {
        while (getline(myfile,line)) {
            if(lineNumber == lineNumberSought) {
                std::istringstream myline(line);
                while(getline(myline, csvItem, ',')) {
                    acc.push_back(std::stof(csvItem));
                }
                break;
            }
            
            lineNumber++;
        }

        myfile.close();
    
    } else {
        std::cerr << "Unable to load file!" << std::endl;
    }
    /*for (auto& val : acc) {
        val *= 9.803f; // Convert to m/s^2
    }*/
    return acc;
}

// Angular velocity
std::vector<float> getOmega(int lineNumberSought) {
    std::string line, csvItem;
    std::vector<float> omega;
    std::ifstream myfile("../csv_measurements/Sensor Simulation/Sensors/omega_sensor.csv");

    int lineNumber = 1;
    if (myfile.is_open()) {
        while (getline(myfile,line)) {
            if(lineNumber == lineNumberSought) {
                std::istringstream myline(line);
                while(getline(myline, csvItem, ',')) {
                    omega.push_back(std::stof(csvItem));
                }
                break;
            }
            
            lineNumber++;
        }

        myfile.close();
    
    } else {
        std::cerr << "Unable to load file!" << std::endl;
    }
    return omega;
}

// Altitude from pressure sensor
std::vector<float> getPressure(int lineNumberSought) {
    std::string line;
    std::vector<float> altitudes;
    std::ifstream myfile("../csv_measurements/Sensor Simulation/Sensors/pressure_sensor.csv");

    int lineNumber = 1;
    if (myfile.is_open()) {
        while (getline(myfile, line)) {
            if (lineNumber == lineNumberSought) {
                try {
                    altitudes.push_back(std::stof(line));
                } catch (const std::invalid_argument& e) {
                    std::cerr << "Invalid float in line: " << line << std::endl;
                }
                break;
            }
            lineNumber++;
        }
        myfile.close();
    } else {
        std::cerr << "Unable to load file!" << std::endl;
    }

    return altitudes;
}

// GPS data
std::vector<float> getGPS(int lineNumberSought) {
    std::string line, csvItem;
    std::vector<float> gps;
    std::ifstream myfile("../csv_measurements/Sensor Simulation/Sensors/z_sensor.csv");

    int lineNumber = 1;
    if (myfile.is_open()) {
        while (getline(myfile,line)) {
            if(lineNumber == lineNumberSought) {
                std::istringstream myline(line);
                while(getline(myline, csvItem, ',')) {
                    gps.push_back(std::stof(csvItem));
                }
                break;
            }
            
            lineNumber++;
        }

        myfile.close();
    
    } else {
        std::cerr << "Unable to load file!" << std::endl;
    }
    return gps;
}

// Save values to a CSV file
void saveToCSV(const std::vector<float>& data, const std::string& filename) {
    std::ofstream file(filename, std::ios::app);

    if (file.is_open()) {
        for (size_t i = 0; i < data.size(); ++i) {
            file << data[i];
            if (i < data.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
        file.close();
    } else {
        std::cerr << "Unable to open file for writing.\n";
    }
}

// Find the number of lines in a file
int countLinesInFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) { 
        std::cerr << "Unable to open file: " << filename << std::endl;
        return 0;
    }
    int lineCount = 0;
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty()) { // Count only non-empty lines
            lineCount++;
        }
    }
    file.close();
    return lineCount;
}

// Initialize the CSV files
void initializeCSVFiles() {
    
    std::ofstream file("../csv_measurements/Sensor Simulation/EKF Results/altitude_EKF.csv", std::ios::trunc);
    file.close();
    std::ofstream file2("../csv_measurements/Sensor Simulation/EKF Results/vel_Z_EKF.csv", std::ios::trunc);
    file2.close();
    std::ofstream file3("../csv_measurements/Sensor Simulation/EKF Results/quat_EKF.csv", std::ios::trunc);
    file3.close();
    std::ofstream file4("../csv_measurements/Sensor Simulation/EKF Results/accel_abs_EKF.csv", std::ios::trunc);
    file4.close();
    std::ofstream file5("../csv_measurements/Sensor Simulation/EKF Results/omega_abs.csv", std::ios::trunc);
    file5.close();
}