#ifndef CSV_UTILS_H
#define CSV_UTILS_H

#include <vector>
#include <string>

// Lectura
std::vector<float> getAcc(int lineNumberSought);
std::vector<float> getOmega(int lineNumberSought);
std::vector<float> getPressure(int lineNumberSought);
std::vector<float> getGPS(int lineNumberSought);

// Escritura
void saveToCSV(const std::vector<float>& data, const std::string& filename);
int countLinesInFile(const std::string& filename);

#endif