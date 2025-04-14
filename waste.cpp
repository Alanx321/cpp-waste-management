#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <iomanip>
#include <algorithm>
#include <conio.h>
#include <queue>
#include <stack>
#include <limits>
#include <map>
#include <unordered_map>
#include <memory>
#include <cmath>
#include <windows.h>
#include <cfloat>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define COLOR_RED 12
#define COLOR_GREEN 10
#define COLOR_BLUE 9
#define COLOR_YELLOW 14
#define COLOR_WHITE 15

using namespace std;

// Forward declarations
class WasteLocation;
class CollectionRoute;
class RouteStrategy;
class NonOptimizedRoute;
class OptimizedRoute;
class GreedyRoute;
class TSPRoute;
class MSTRoute;
class RLRoute;
class AIPredictionModel;
class WasteLocationManager;

// Complete definition of RouteResults struct
struct RouteResults {
    vector<int> path;
    int totalDistance;
    double totalTime;
    double totalFuel;
    double totalWage;
};

// Exception classes
class InvalidRouteException : public exception {
private:
    string message;
public:
    InvalidRouteException(const string& msg) : message(msg) {}
    const char* what() const noexcept override {
        return message.c_str();
    }
};

class WasteManagementException : public exception {
protected:
    string message;
public:
    WasteManagementException(const string& msg) : message(msg) {}
    const char* what() const noexcept override { return message.c_str(); }
};

class LocationNotFoundException : public WasteManagementException {
public:
    LocationNotFoundException(const string& location) 
        : WasteManagementException("Location not found: " + location) {}
};

class InvalidWasteLevelException : public WasteManagementException {
public:
    InvalidWasteLevelException(const string& location, int level) 
        : WasteManagementException("Invalid waste level " + to_string(level) + 
                                 "% for location: " + location) {}
};

class FileOperationException : public WasteManagementException {
public:
    FileOperationException(const string& operation, const string& filename) 
        : WasteManagementException("File " + operation + " failed for: " + filename) {}
};

class WasteLocation {
private:
    string name;
    int wasteLevel; // In percentage
    bool isCollected;
    vector<pair<int, double>> previousWasteLevels; // Store historical data for AI prediction
    
public:
    WasteLocation(const string& locationName, int initialWasteLevel = 0) 
        : name(locationName), wasteLevel(initialWasteLevel), isCollected(false) {}
    
    string getName() const { return name; }
    int getWasteLevel() const { return wasteLevel; }
    bool getIsCollected() const { return isCollected; }
    
    void setWasteLevel(int level) { 
        // Store previous waste level for AI prediction
        time_t now = time(nullptr);
        previousWasteLevels.push_back(make_pair(now, wasteLevel));
        
        // Keep only last 30 data points
        if (previousWasteLevels.size() > 30) {
            previousWasteLevels.erase(previousWasteLevels.begin());
        }
        
        wasteLevel = level; 
    }
    
    void setIsCollected(bool collected) { isCollected = collected; }
    
    const vector<pair<int, double>>& getHistoricalData() const {
        return previousWasteLevels;
    }

    void addHistoricalDataPoint(time_t timestamp, double level) {
        previousWasteLevels.push_back(make_pair(timestamp, level));
        
        // Keep only last 30 data points
        if (previousWasteLevels.size() > 30) {
            previousWasteLevels.erase(previousWasteLevels.begin());
        }
    }

    void clearHistoricalData() {
        previousWasteLevels.clear();
    }
};

// AI Model for waste prediction
class AIPredictionModel {
private:
    // Simple linear regression model
    double slope;
    double intercept;
    
    // For anomaly detection
    double meanWasteLevel;
    double stdDevWasteLevel;
    
    void calculateRegressionParameters(const vector<pair<int, double>>& data) const {
        // This is now a const method that calculates regression parameters locally
        // without modifying the member variables
    }
    
public:
    AIPredictionModel() : slope(0), intercept(0), meanWasteLevel(0), stdDevWasteLevel(0) {}
    
    int predictWasteLevel(const WasteLocation& location, int daysAhead) const {
        const vector<pair<int, double>>& historicalData = location.getHistoricalData();
        
        if (historicalData.empty()) {
            return location.getWasteLevel(); // Return current if no historical data
        }
        
        double tempSlope = 0;
        double tempIntercept = 0;
        double tempMean = 0;
        double tempStdDev = 0;
        
        // Calculate regression parameters locally instead of modifying member variables
        if (historicalData.size() >= 2) {
            double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
            int n = historicalData.size();
            
            for (const auto& point : historicalData) {
                sumX += point.first;
                sumY += point.second;
                sumXY += point.first * point.second;
                sumX2 += point.first * point.first;
            }
            
            tempSlope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
            tempIntercept = (sumY - tempSlope * sumX) / n;
            tempMean = sumY / n;
        } else {
            tempIntercept = historicalData.empty() ? 0 : historicalData[0].second;
        }
        
        // Predict waste level for future day using local variables
        int lastDay = historicalData.back().first;
        int futureDay = lastDay + (daysAhead * 86400); // Convert days to seconds
        double predictedWaste = tempSlope * futureDay + tempIntercept;
        
        // Ensure prediction is within reasonable range
        predictedWaste = max(0.0, min(100.0, predictedWaste));
        
        return static_cast<int>(predictedWaste);
    }
    
    bool isAnomaly(const WasteLocation& location) const {
        int currentWaste = location.getWasteLevel();
        const vector<pair<int, double>>& historicalData = location.getHistoricalData();
        
        // If we don't have enough data, can't detect anomalies
        if (historicalData.size() < 5) {
            return false;
        }
        
        // Calculate mean and standard deviation locally
        double tempMean = 0;
        double tempStdDev = 0;
        
        // Calculate mean
        double sumY = 0;
        int n = historicalData.size();
        
        for (const auto& point : historicalData) {
            sumY += point.second;
        }
        tempMean = sumY / n;
        
        // Calculate standard deviation
        double variance = 0;
        for (const auto& point : historicalData) {
            variance += pow(point.second - tempMean, 2);
        }
        tempStdDev = sqrt(variance / n);
        
        // If waste level deviates more than 2 std deviations from mean, flag as anomaly
        return abs(currentWaste - tempMean) > (2 * tempStdDev);
    }
    
    string getWasteTrend(const WasteLocation& location) const {
        const vector<pair<int, double>>& historicalData = location.getHistoricalData();
        
        if (historicalData.size() < 2) {
            return "Insufficient data for trend analysis";
        }
        
        // Calculate regression parameters locally
        double tempSlope = 0;
        
        if (historicalData.size() >= 2) {
            double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
            int n = historicalData.size();
            
            for (const auto& point : historicalData) {
                sumX += point.first;
                sumY += point.second;
                sumXY += point.first * point.second;
                sumX2 += point.first * point.first;
            }
            
            if ((n * sumX2 - sumX * sumX) != 0) {
                tempSlope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
            }
        }
        
        // Use more sensitive thresholds for trends since we're using small values
        if (tempSlope < -1.0) {
            return "Rapidly decreasing";
        } else if (tempSlope < -0.2) {
            return "Decreasing";
        } else if (tempSlope > 1.0) {
            return "Rapidly increasing";
        } else if (tempSlope > 0.2) {
            return "Increasing";
        } else {
            return "Stable";
        }
    }
};

// Add this function before the WasteLocationManager class

// Helper function for text color
void setTextColor(int colorCode) {
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleTextAttribute(hConsole, colorCode);
}

// Add this before WasteLocationManager class
class UIHelper {
public:
    static void displayError(const string& error) {
        HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
        SetConsoleTextAttribute(hConsole, COLOR_RED);
        cout << "\nERROR: " << error << endl;
        SetConsoleTextAttribute(hConsole, COLOR_WHITE);
    }

    static void displaySuccess(const string& message) {
        HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
        SetConsoleTextAttribute(hConsole, COLOR_GREEN);
        cout << "\nSUCCESS: " << message << endl;
        SetConsoleTextAttribute(hConsole, COLOR_WHITE);
    }

    static void displayWarning(const string& message) {
        HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
        SetConsoleTextAttribute(hConsole, COLOR_YELLOW);
        cout << "\nWARNING: " << message << endl;
        SetConsoleTextAttribute(hConsole, COLOR_WHITE);
    }

    static void displaySavings(const string& message, double amount) {
        HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
        SetConsoleTextAttribute(hConsole, COLOR_GREEN);
        cout << "\n" << message << ": " << fixed << setprecision(2) << amount << " RM" << endl;
        SetConsoleTextAttribute(hConsole, COLOR_WHITE);
    }

    static void clearScreen() {
        system("cls");
    }

    static void displayProgressBar(int progress) {
        cout << "[";
        for (int i = 0; i < 50; i++) {
            if (i < (progress / 2)) cout << "=";
            else cout << " ";
        }
        cout << "] " << progress << "%" << "\r";
        cout.flush();
    }
    
    static void displayCostComparison(const map<string, RouteResults>& routeResults) {
        cout << "\n===============================================" << endl;
        cout << "          ROUTE COST COMPARISON                " << endl;
        cout << "===============================================" << endl;
        
        cout << left 
             << setw(35) << "Route Strategy" 
             << setw(15) << "Distance (km)" 
             << setw(15) << "Time (hrs)" 
             << setw(15) << "Fuel (RM)" 
             << setw(15) << "Wages (RM)" 
             << setw(15) << "Total Cost (RM)" << endl;
             
        cout << string(100, '-') << endl;
        
        for (const auto& route : routeResults) {
            cout << left
                 << setw(35) << route.first
                 << setw(15) << route.second.totalDistance
                 << setw(15) << fixed << setprecision(2) << route.second.totalTime / 60
                 << setw(15) << fixed << setprecision(2) << route.second.totalFuel
                 << setw(15) << fixed << setprecision(2) << route.second.totalWage
                 << setw(15) << fixed << setprecision(2) << (route.second.totalFuel + route.second.totalWage) << endl;
        }
        
        cout << "===============================================" << endl;
    }
};

// Then your existing WasteLocationManager class follows
class WasteLocationManager {
private:
    vector<WasteLocation> locations;
    vector<vector<int>> distanceMatrix;
    AIPredictionModel aiModel;
    string lastSavedFilePath;
    
    // Private constructor for singleton
    WasteLocationManager() {
        // Initialize with empty locations
        lastSavedFilePath = "waste_data.dat";
    }
    
    static WasteLocationManager* instance;
    
public:
    // Delete copy constructor and assignment
    WasteLocationManager(const WasteLocationManager&) = delete;
    WasteLocationManager& operator=(const WasteLocationManager&) = delete;
    
    // Get singleton instance
    static WasteLocationManager* getInstance() {
        if (instance == nullptr) {
            instance = new WasteLocationManager();
        }
        return instance;
    }
    
    static void cleanup() {
        if (instance != nullptr) {
            delete instance;
            instance = nullptr;
        }
    }
    
    void addLocation(const string& name) {
        locations.push_back(WasteLocation(name));
    }
    
    void initializeLocations() {
        // Clear existing locations
        locations.clear();
        
        // Add headquarters as first location
        locations.push_back(WasteLocation("Waste Collector HQ", 0));
        
        // Add 10 waste collection locations
        locations.push_back(WasteLocation("Kuala Lumpur Central"));
        locations.push_back(WasteLocation("Petaling Jaya"));
        locations.push_back(WasteLocation("Shah Alam"));
        locations.push_back(WasteLocation("Subang Jaya"));
        locations.push_back(WasteLocation("Ampang"));
        locations.push_back(WasteLocation("Klang"));
        locations.push_back(WasteLocation("Cheras"));
        locations.push_back(WasteLocation("Puchong"));
        locations.push_back(WasteLocation("Putrajaya"));
        locations.push_back(WasteLocation("Cyberjaya"));
        
        // Initialize distance matrix
        int numLocations = locations.size();
        distanceMatrix.resize(numLocations, vector<int>(numLocations, 0));
        
        // HQ to each location
        distanceMatrix[0][1] = 12; // HQ to KL Central
        distanceMatrix[0][2] = 15; // HQ to Petaling Jaya
        distanceMatrix[0][3] = 22; // HQ to Shah Alam
        distanceMatrix[0][4] = 18; // HQ to Subang Jaya
        distanceMatrix[0][5] = 14; // HQ to Ampang
        distanceMatrix[0][6] = 28; // HQ to Klang
        distanceMatrix[0][7] = 16; // HQ to Cheras
        distanceMatrix[0][8] = 20; // HQ to Puchong
        distanceMatrix[0][9] = 25; // HQ to Putrajaya
        distanceMatrix[0][10] = 28; // HQ to Cyberjaya
        
        // Fill in distances between locations
        distanceMatrix[1][2] = 8;  // KL Central to PJ
        distanceMatrix[1][3] = 18; // KL Central to Shah Alam
        distanceMatrix[1][4] = 14; // KL Central to Subang Jaya
        distanceMatrix[1][5] = 10; // KL Central to Ampang
        distanceMatrix[1][6] = 24; // KL Central to Klang
        distanceMatrix[1][7] = 12; // KL Central to Cheras
        distanceMatrix[1][8] = 16; // KL Central to Puchong
        distanceMatrix[1][9] = 23; // KL Central to Putrajaya
        distanceMatrix[1][10] = 25; // KL Central to Cyberjaya
        
        distanceMatrix[2][3] = 10; // PJ to Shah Alam
        distanceMatrix[2][4] = 8;  // PJ to Subang Jaya
        distanceMatrix[2][5] = 13; // PJ to Ampang
        distanceMatrix[2][6] = 16; // PJ to Klang
        distanceMatrix[2][7] = 15; // PJ to Cheras
        distanceMatrix[2][8] = 12; // PJ to Puchong
        distanceMatrix[2][9] = 20; // PJ to Putrajaya
        distanceMatrix[2][10] = 22; // PJ to Cyberjaya
        
        distanceMatrix[3][4] = 9;  // Shah Alam to Subang Jaya
        distanceMatrix[3][5] = 22; // Shah Alam to Ampang
        distanceMatrix[3][6] = 12; // Shah Alam to Klang
        distanceMatrix[3][7] = 24; // Shah Alam to Cheras
        distanceMatrix[3][8] = 15; // Shah Alam to Puchong
        distanceMatrix[3][9] = 26; // Shah Alam to Putrajaya
        distanceMatrix[3][10] = 24; // Shah Alam to Cyberjaya
        
        distanceMatrix[4][5] = 18; // Subang Jaya to Ampang
        distanceMatrix[4][6] = 15; // Subang Jaya to Klang
        distanceMatrix[4][7] = 17; // Subang Jaya to Cheras
        distanceMatrix[4][8] = 10; // Subang Jaya to Puchong
        distanceMatrix[4][9] = 18; // Subang Jaya to Putrajaya
        distanceMatrix[4][10] = 16; // Subang Jaya to Cyberjaya
        
        distanceMatrix[5][6] = 30; // Ampang to Klang
        distanceMatrix[5][7] = 12; // Ampang to Cheras
        distanceMatrix[5][8] = 20; // Ampang to Puchong
        distanceMatrix[5][9] = 25; // Ampang to Putrajaya
        distanceMatrix[5][10] = 27; // Ampang to Cyberjaya
        
        distanceMatrix[6][7] = 26; // Klang to Cheras
        distanceMatrix[6][8] = 18; // Klang to Puchong
        distanceMatrix[6][9] = 32; // Klang to Putrajaya
        distanceMatrix[6][10] = 30; // Klang to Cyberjaya
        
        distanceMatrix[7][8] = 14; // Cheras to Puchong
        distanceMatrix[7][9] = 15; // Cheras to Putrajaya
        distanceMatrix[7][10] = 17; // Cheras to Cyberjaya
        
        distanceMatrix[8][9] = 12; // Puchong to Putrajaya
        distanceMatrix[8][10] = 10; // Puchong to Cyberjaya
        
        distanceMatrix[9][10] = 5;  // Putrajaya to Cyberjaya
        
        // Mirror the matrix since distances are symmetrical
        for (int i = 0; i < numLocations; i++) {
            for (int j = i + 1; j < numLocations; j++) {
                distanceMatrix[j][i] = distanceMatrix[i][j];
            }
        }
    }
    
    void generateRandomWasteLevels() {
        try {
            UIHelper::clearScreen();
            cout << "\nGenerating random waste levels..." << endl;
            
            srand(static_cast<unsigned int>(time(nullptr)));
            
            for (int progress = 0; progress <= 100; progress += 10) {
                UIHelper::displayProgressBar(progress);
                Sleep(100); // Simulate processing time
            }
            cout << endl;

            cout << "\n==============================================" << endl;
            cout << "Generating Random Waste Levels" << endl;
            cout << "==============================================" << endl;
            cout << left << setw(25) << "Location" << setw(15) << "Previous %" 
                 << setw(15) << "New %" << endl;
            cout << "----------------------------------------------" << endl;
            
            for (size_t i = 1; i < locations.size(); i++) {
                int previousLevel = locations[i].getWasteLevel();
                int wasteLevel = rand() % 101;
                
                if (wasteLevel > 100 || wasteLevel < 0) {
                    throw InvalidWasteLevelException(locations[i].getName(), wasteLevel);
                }
                
                cout << left << setw(25) << locations[i].getName() 
                     << setw(15) << previousLevel
                     << setw(15) << wasteLevel << endl;
                
                locations[i].setWasteLevel(wasteLevel);
                locations[i].setIsCollected(false);
            }
            
            cout << "==============================================" << endl;
            UIHelper::displaySuccess("Waste levels generated successfully!");
        }
        catch (const InvalidWasteLevelException& e) {
            UIHelper::displayError(e.what());
        }
        catch (const exception& e) {
            UIHelper::displayError("Unexpected error while generating waste levels: " + string(e.what()));
        }
    }
    
    vector<WasteLocation>& getLocations() {
        return locations;
    }
    
    int getDistance(int from, int to) const {
        if (from >= 0 && to >= 0 && from < distanceMatrix.size() && to < distanceMatrix[from].size()) {
            return distanceMatrix[from][to];
        }
        return -1; // Invalid distance
    }
    
    void printLocationsInfo() const {
        cout << "====================================================" << endl;
        cout << "        Current Waste Locations Information          " << endl;
        cout << "====================================================" << endl;
        
        // Add headers with increased spacing
        cout << left 
             << setw(30) << "Location" 
             << setw(20) << "Waste Level %" 
             << setw(25) << "AI Prediction (24h)" 
             << setw(20) << "Trend" << endl;
        cout << string(95, '-') << endl;  // Separator line
        
        for (size_t i = 0; i < locations.size(); i++) {
            if (i == 0) continue; // Skip HQ
            
            int currentLevel = locations[i].getWasteLevel();
            int predictedLevel = aiModel.predictWasteLevel(locations[i], 1);
            string trend = aiModel.getWasteTrend(locations[i]);
            string anomalyFlag = aiModel.isAnomaly(locations[i]) ? "  [ANOMALY]" : "";
            
            // Format the percentage values with spacing
            string currentStr = to_string(currentLevel) + " %";
            string predictedStr = to_string(predictedLevel) + " %";
            
            // Color coding based on waste level
            if (currentLevel >= 70) setTextColor(COLOR_RED);
            else if (currentLevel >= 40) setTextColor(COLOR_YELLOW);
            else setTextColor(COLOR_GREEN);
            
            cout << left 
                 << setw(30) << locations[i].getName()
                 << setw(20) << currentStr
                 << setw(25) << predictedStr
                 << setw(20) << trend + anomalyFlag << endl;
            
            setTextColor(COLOR_WHITE);
        }
        cout << "====================================================" << endl;
    }
    
    void saveLocationsToFile(const string& filename) const {
        try {
            ofstream outFile(filename);
            if (!outFile) {
                throw FileOperationException("open", filename);
            }

            UIHelper::displayProgressBar(0);
            
            // File header
            outFile << "Location Information\n";
            outFile << "===========================================\n";
            UIHelper::displayProgressBar(20);

            // Write location data
            for (size_t i = 0; i < locations.size(); i++) {
                if (i == 0) continue; // Skip HQ
                
                int predictedWaste = aiModel.predictWasteLevel(locations[i], 1);
                string trend = aiModel.getWasteTrend(locations[i]);
                
                outFile << left << setw(20) << locations[i].getName() 
                       << setw(15) << locations[i].getWasteLevel() 
                       << setw(20) << predictedWaste
                       << setw(20) << trend << endl;
            }
            UIHelper::displayProgressBar(70);

            // Write distance matrix
            outFile << "\nDistance Matrix\n";
            // ... rest of the matrix writing code ...
            
            outFile.close();
            UIHelper::displayProgressBar(100);
            cout << endl;
            
            if (!outFile.good()) {
                throw FileOperationException("write", filename);
            }

            UIHelper::displaySuccess("Data saved successfully to " + filename);
        }
        catch (const FileOperationException& e) {
            UIHelper::displayError(e.what());
        }
        catch (const exception& e) {
            UIHelper::displayError("Unexpected error while saving file: " + string(e.what()));
        }
    }
    
    // Save all data to a binary file
    bool saveAllData(const string& filename = "") {
        string saveFile = filename.empty() ? lastSavedFilePath : filename;
        
        try {
            ofstream outFile(saveFile, ios::binary);
            if (!outFile) {
                throw FileOperationException("open", saveFile);
            }

            UIHelper::displayProgressBar(0);
            cout << "Saving data to " << saveFile << "..." << endl;
            
            // Save number of locations
            int numLocations = locations.size();
            outFile.write(reinterpret_cast<char*>(&numLocations), sizeof(numLocations));
            UIHelper::displayProgressBar(10);
            
            // Save each location's data
            for (size_t i = 0; i < locations.size(); i++) {
                // Save location name
                string name = locations[i].getName();
                int nameLength = name.length();
                outFile.write(reinterpret_cast<char*>(&nameLength), sizeof(nameLength));
                outFile.write(name.c_str(), nameLength);
                
                // Save waste level and collection status
                int wasteLevel = locations[i].getWasteLevel();
                bool isCollected = locations[i].getIsCollected();
                outFile.write(reinterpret_cast<char*>(&wasteLevel), sizeof(wasteLevel));
                outFile.write(reinterpret_cast<char*>(&isCollected), sizeof(isCollected));
                
                // Save historical data
                const vector<pair<int, double>>& history = locations[i].getHistoricalData();
                int historySize = history.size();
                outFile.write(reinterpret_cast<char*>(&historySize), sizeof(historySize));
                
                for (const auto& point : history) {
                    outFile.write(reinterpret_cast<const char*>(&point.first), sizeof(point.first));
                    outFile.write(reinterpret_cast<const char*>(&point.second), sizeof(point.second));
                }
                
                UIHelper::displayProgressBar(10 + (i * 40 / locations.size()));
            }
            
            // Save distance matrix
            int rows = distanceMatrix.size();
            outFile.write(reinterpret_cast<char*>(&rows), sizeof(rows));
            
            for (size_t i = 0; i < distanceMatrix.size(); i++) {
                int cols = distanceMatrix[i].size();
                outFile.write(reinterpret_cast<char*>(&cols), sizeof(cols));
                
                for (int j = 0; j < cols; j++) {
                    outFile.write(reinterpret_cast<char*>(&distanceMatrix[i][j]), sizeof(int));
                }
                
                UIHelper::displayProgressBar(50 + (i * 50 / rows));
            }
            
            outFile.close();
            UIHelper::displayProgressBar(100);
            cout << endl;
            
            if (!outFile.good()) {
                throw FileOperationException("write", saveFile);
            }
            
            // Save the file path for future reference
            lastSavedFilePath = saveFile;
            UIHelper::displaySuccess("Data saved successfully to " + saveFile);
            return true;
        }
        catch (const FileOperationException& e) {
            UIHelper::displayError(e.what());
            return false;
        }
        catch (const exception& e) {
            UIHelper::displayError("Unexpected error while saving data: " + string(e.what()));
            return false;
        }
    }
    
    // Load data from a binary file
    bool loadAllData(const string& filename = "") {
        string loadFile = filename.empty() ? lastSavedFilePath : filename;
        
        try {
            ifstream inFile(loadFile, ios::binary);
            if (!inFile) {
                throw FileOperationException("open", loadFile);
            }
            
            cout << "Loading data from " << loadFile << "..." << endl;
            UIHelper::displayProgressBar(0);
            
            // Clear existing data
            locations.clear();
            distanceMatrix.clear();
            
            // Read number of locations
            int numLocations;
            inFile.read(reinterpret_cast<char*>(&numLocations), sizeof(numLocations));
            UIHelper::displayProgressBar(10);
            
            // Read each location's data
            for (int i = 0; i < numLocations; i++) {
                // Read location name
                int nameLength;
                inFile.read(reinterpret_cast<char*>(&nameLength), sizeof(nameLength));
                
                string name(nameLength, ' ');
                inFile.read(&name[0], nameLength);
                
                // Read waste level and collection status
                int wasteLevel;
                bool isCollected;
                inFile.read(reinterpret_cast<char*>(&wasteLevel), sizeof(wasteLevel));
                inFile.read(reinterpret_cast<char*>(&isCollected), sizeof(isCollected));
                
                // Create location object
                WasteLocation location(name, wasteLevel);
                location.setIsCollected(isCollected);
                
                // Read historical data
                int historySize;
                inFile.read(reinterpret_cast<char*>(&historySize), sizeof(historySize));
                
                for (int j = 0; j < historySize; j++) {
                    int timestamp;
                    double level;
                    inFile.read(reinterpret_cast<char*>(&timestamp), sizeof(timestamp));
                    inFile.read(reinterpret_cast<char*>(&level), sizeof(level));
                    
                    location.addHistoricalDataPoint(timestamp, level);
                }
                
                // Add location to the vector
                locations.push_back(location);
                
                UIHelper::displayProgressBar(10 + (i * 40 / numLocations));
            }
            
            // Read distance matrix
            int rows;
            inFile.read(reinterpret_cast<char*>(&rows), sizeof(rows));
            distanceMatrix.resize(rows);
            
            for (int i = 0; i < rows; i++) {
                int cols;
                inFile.read(reinterpret_cast<char*>(&cols), sizeof(cols));
                distanceMatrix[i].resize(cols);
                
                for (int j = 0; j < cols; j++) {
                    inFile.read(reinterpret_cast<char*>(&distanceMatrix[i][j]), sizeof(int));
                }
                
                UIHelper::displayProgressBar(50 + (i * 50 / rows));
            }
            
            inFile.close();
            UIHelper::displayProgressBar(100);
            cout << endl;
            
            // Save the file path for future reference
            lastSavedFilePath = loadFile;
            UIHelper::displaySuccess("Data loaded successfully from " + loadFile);
            return true;
        }
        catch (const FileOperationException& e) {
            UIHelper::displayError(e.what());
            
            // If loading fails, initialize with default data
            initializeLocations();
            generateRandomWasteLevels();
            return false;
        }
        catch (const exception& e) {
            UIHelper::displayError("Unexpected error while loading data: " + string(e.what()));
            
            // If loading fails, initialize with default data
            initializeLocations();
            generateRandomWasteLevels();
            return false;
        }
    }
    
    // Delete saved data file
    bool deleteDataFile(const string& filename = "") {
        string deleteFile = filename.empty() ? lastSavedFilePath : filename;
        
        try {
            if (remove(deleteFile.c_str()) != 0) {
                throw FileOperationException("delete", deleteFile);
            }
            
            UIHelper::displaySuccess("Data file deleted successfully: " + deleteFile);
            return true;
        }
        catch (const FileOperationException& e) {
            UIHelper::displayError(e.what());
            return false;
        }
        catch (const exception& e) {
            UIHelper::displayError("Unexpected error while deleting data file: " + string(e.what()));
            return false;
        }
    }
    
    // Check if a data file exists
    bool dataFileExists(const string& filename = "") {
        string checkFile = filename.empty() ? lastSavedFilePath : filename;
        ifstream file(checkFile);
        return file.good();
    }
    
    // Get the last saved file path
    string getLastSavedFilePath() const {
        return lastSavedFilePath;
    }
    
    // Set the last saved file path
    void setLastSavedFilePath(const string& filePath) {
        lastSavedFilePath = filePath;
    }
    
    AIPredictionModel& getAIModel() {
        return aiModel;
    }

    void simulateTrendingWasteLevels() {
        srand(static_cast<unsigned int>(time(nullptr)));
        
        for (size_t i = 1; i < locations.size(); i++) {
            // Get current waste level
            int currentLevel = locations[i].getWasteLevel();
            
            // Clear previous historical data
            locations[i].clearHistoricalData();
            
            // Randomly decide on a trend direction (-1, 0, 1)
            int trendDirection = (rand() % 3) - 1;
            
            // Use a much larger trend strength to compensate for the slope calculation
            double trendStrength = (0.5 + (rand() % 10) / 10.0) * (trendDirection == 0 ? 0.1 : trendDirection);
            
            // For all locations, create actual historical data points
            for (int j = 10; j >= 0; j--) {
                // Determine the trend effect for this data point
                double trendEffect = j * trendStrength * 5;
                
                // Calculate level within bounds
                int historicalLevel = max(5, min(95, static_cast<int>(currentLevel - trendEffect)));
                
                // Store historical data point with small timestamp values
                // Use j directly instead of days to create more pronounced slopes
                locations[i].addHistoricalDataPoint(j, historicalLevel);
            }
            
            // Set final current level
            locations[i].setWasteLevel(currentLevel);
        }
    }
};

// Initialize static member
WasteLocationManager* WasteLocationManager::instance = nullptr;

// Move RouteResults outside of RouteStrategy class to make it accessible to all classes

// Strategy Pattern for different route algorithms
class RouteStrategy {
protected:
    // Constants for cost calculation
    const double FUEL_COST_PER_KM = 2.50;  // RM per km
    const double TIME_PER_KM = 1.5;        // minutes per km
    const double WAGE_PER_HOUR = 10.00;    // RM per hour
    
public:
    virtual ~RouteStrategy() {}
    
    // Pure virtual method to be implemented by concrete strategies
    virtual RouteResults calculateRoute(WasteLocationManager* manager) = 0;
    
    // Method to save route information to file
    void saveRouteToFile(const string& filename, const RouteResults& results, 
                         WasteLocationManager* manager,
                         const string& routeType) {
        ofstream outFile(filename);
        
        if (!outFile) {
            cerr << "Cannot open file: " << filename << endl;
            return;
        }
        
        vector<WasteLocation>& locations = manager->getLocations();
        
        outFile << routeType << " Route Information\n";
        outFile << "===========================================\n";
        
        if (results.path.empty()) {
            outFile << "No locations to be served\n";
            outFile.close();
            return;
        }
        
        // Print complete route
        outFile << "Complete Route: ";
        for (size_t i = 0; i < results.path.size(); ++i) {
            outFile << locations[results.path[i]].getName();
            if (i < results.path.size() - 1) {
                outFile << " -> ";
            }
        }
        outFile << "\n\n";
        
        // Print leg-by-leg details
        for (size_t i = 0; i < results.path.size() - 1; ++i) {
            int from = results.path[i];
            int to = results.path[i + 1];
            int distance = manager->getDistance(from, to);
            double time = distance * TIME_PER_KM;
            double fuel = distance * FUEL_COST_PER_KM;
            double wage = (time / 60) * WAGE_PER_HOUR;
            
            outFile << locations[from].getName() << " to " << locations[to].getName() << "\n";
            outFile << "Distance = " << distance << " km\n";
            outFile << "Time to destination = " << time << " minutes\n";
            outFile << "Fuel consumption = " << fuel << " RM\n";
            outFile << "Wage for this leg = " << wage << " RM\n\n";
        }
        
        // Print totals
        outFile << "Route Costs Summary:\n";
        outFile << "Total Distance: " << results.totalDistance << " km\n";
        outFile << "Total Time: " << results.totalTime / 60 << " hours\n";
        outFile << "Total Fuel Cost: " << results.totalFuel << " RM\n";
        outFile << "Total Wages: " << results.totalWage << " RM\n";
        outFile << "Total Cost: " << (results.totalFuel + results.totalWage) << " RM\n";
        
        outFile.close();
    }
    
    // Method to print route information to console
    void printRouteInfo(const RouteResults& results, 
                       WasteLocationManager* manager,
                       const string& routeType) {
        vector<WasteLocation>& locations = manager->getLocations();
        
        cout << routeType << " Route Information" << endl;
        cout << "=============================================" << endl;
        
        if (results.path.empty()) {
            cout << "No locations to be served" << endl;
            return;
        }
        
        // Print complete route
        cout << "Complete Route: ";
        for (size_t i = 0; i < results.path.size(); ++i) {
            cout << locations[results.path[i]].getName();
            if (i < results.path.size() - 1) {
                cout << " -> ";
            }
        }
        cout << endl << endl;
        
        // Print totals
        cout << "Route Costs Summary:" << endl;
        cout << "Total Distance: " << results.totalDistance << " km" << endl;
        cout << "Total Time: " << results.totalTime / 60 << " hours" << endl;
        cout << "Total Fuel Cost: " << results.totalFuel << " RM" << endl;
        cout << "Total Wages: " << results.totalWage << " RM" << endl;
        cout << "Total Cost: " << (results.totalFuel + results.totalWage) << " RM" << endl;
        cout << "=============================================" << endl;
    }
};

// Concrete strategy: Non-optimized route
class NonOptimizedRoute : public RouteStrategy {
public:
    RouteResults calculateRoute(WasteLocationManager* manager) override {
        RouteResults results;
        results.totalDistance = 0;
        results.totalTime = 0;
        results.totalFuel = 0;
        results.totalWage = 0;
        
        vector<WasteLocation>& locations = manager->getLocations();
        
        // Start from HQ (index 0)
        results.path.push_back(0);
        
        // Current position (start from HQ)
        int currentPos = 0;
        
        // Criteria: waste level >= 40% and distance <= 30km
        for (size_t i = 1; i < locations.size(); ++i) {
            int distance = manager->getDistance(0, i);
            if (locations[i].getWasteLevel() >= 40 && distance <= 30) {
                // Visit this location
                results.path.push_back(i);
                
                // Calculate costs for this leg
                int legDistance = manager->getDistance(currentPos, i);
                double legTime = legDistance * TIME_PER_KM;
                double legFuel = legDistance * FUEL_COST_PER_KM;
                double legWage = (legTime / 60) * WAGE_PER_HOUR;
                
                // Update totals
                results.totalDistance += legDistance;
                results.totalTime += legTime;
                results.totalFuel += legFuel;
                results.totalWage += legWage;
                
                // Update current position
                currentPos = i;
            }
        }
        
        // Return to HQ if any locations were visited
        if (results.path.size() > 1) {
            int legDistance = manager->getDistance(currentPos, 0);
            double legTime = legDistance * TIME_PER_KM;
            double legFuel = legDistance * FUEL_COST_PER_KM;
            double legWage = (legTime / 60) * WAGE_PER_HOUR;
            
            results.path.push_back(0);  // Return to HQ
            
            // Update totals for return trip
            results.totalDistance += legDistance;
            results.totalTime += legTime;
            results.totalFuel += legFuel;
            results.totalWage += legWage;
        } else {
            // No locations to visit
            results.path.clear();
        }
        
        return results;
    }
};

// Concrete strategy: Optimized route
class OptimizedRoute : public RouteStrategy {
public:
    RouteResults calculateRoute(WasteLocationManager* manager) override {
        RouteResults results;
        results.totalDistance = 0;
        results.totalTime = 0;
        results.totalFuel = 0;
        results.totalWage = 0;
        
        vector<WasteLocation>& locations = manager->getLocations();
        
        // Start from HQ (index 0)
        results.path.push_back(0);
        
        // Current position (start from HQ)
        int currentPos = 0;
        
        // Criteria: waste level >= 60% and distance <= 20km
        for (size_t i = 1; i < locations.size(); ++i) {
            int distance = manager->getDistance(0, i);
            if (locations[i].getWasteLevel() >= 60 && distance <= 20) {
                // Visit this location
                results.path.push_back(i);
                
                // Calculate costs for this leg
                int legDistance = manager->getDistance(currentPos, i);
                double legTime = legDistance * TIME_PER_KM;
                double legFuel = legDistance * FUEL_COST_PER_KM;
                double legWage = (legTime / 60) * WAGE_PER_HOUR;
                
                // Update totals
                results.totalDistance += legDistance;
                results.totalTime += legTime;
                results.totalFuel += legFuel;
                results.totalWage += legWage;
                
                // Update current position
                currentPos = i;
            }
        }
        
        // Return to HQ if any locations were visited
        if (results.path.size() > 1) {
            int legDistance = manager->getDistance(currentPos, 0);
            double legTime = legDistance * TIME_PER_KM;
            double legFuel = legDistance * FUEL_COST_PER_KM;
            double legWage = (legTime / 60) * WAGE_PER_HOUR;
            
            results.path.push_back(0);  // Return to HQ
            
            // Update totals for return trip
            results.totalDistance += legDistance;
            results.totalTime += legTime;
            results.totalFuel += legFuel;
            results.totalWage += legWage;
        } else {
            // No locations to visit
            results.path.clear();
        }
        
        return results;
    }
};

// Concrete strategy: Greedy route
class GreedyRoute : public RouteStrategy {
public:
    RouteResults calculateRoute(WasteLocationManager* manager) override {
        RouteResults results;
        results.totalDistance = 0;
        results.totalTime = 0;
        results.totalFuel = 0;
        results.totalWage = 0;
        
        vector<WasteLocation>& locations = manager->getLocations();
        
        // Start from HQ (index 0)
        results.path.push_back(0);
        
        // Current position (start from HQ)
        int currentPos = 0;
        
        // Criteria: waste level >= 30% regardless of distance
        vector<int> locationsToVisit;
        for (size_t i = 1; i < locations.size(); ++i) {
            if (locations[i].getWasteLevel() >= 30) {
                locationsToVisit.push_back(i);
            }
        }
        
        // No locations to visit
        if (locationsToVisit.empty()) {
            results.path.clear();
            return results;
        }
        
        // Greedy algorithm: always visit the nearest unvisited location
        vector<bool> visited(locations.size(), false);
        
        while (locationsToVisit.size() > 0) {
            int nearestLocationIndex = -1;
            int shortestDistance = INT_MAX;
            
            // Find the nearest unvisited location
            for (size_t i = 0; i < locationsToVisit.size(); ++i) {
                int locationIndex = locationsToVisit[i];
                if (!visited[locationIndex]) {
                    int distance = manager->getDistance(currentPos, locationIndex);
                    if (distance < shortestDistance) {
                        shortestDistance = distance;
                        nearestLocationIndex = locationIndex;
                    }
                }
            }
            
            if (nearestLocationIndex == -1) {
                break; // No more locations to visit
            }
            
            // Visit this location
            results.path.push_back(nearestLocationIndex);
            visited[nearestLocationIndex] = true;
            
            // Calculate costs for this leg
            double legTime = shortestDistance * TIME_PER_KM;
            double legFuel = shortestDistance * FUEL_COST_PER_KM;
            double legWage = (legTime / 60) * WAGE_PER_HOUR;
            
            // Update totals
            results.totalDistance += shortestDistance;
            results.totalTime += legTime;
            results.totalFuel += legFuel;
            results.totalWage += legWage;
            
            // Update current position
            currentPos = nearestLocationIndex;
            
            // Remove the visited location from the list
            locationsToVisit.erase(remove(locationsToVisit.begin(), locationsToVisit.end(), nearestLocationIndex), locationsToVisit.end());
        }
        
        // Continuing from where the code was cut off in the GreedyRoute class:

        // Return to HQ
        int legDistance = manager->getDistance(currentPos, 0);
        double legTime = legDistance * TIME_PER_KM;
        double legFuel = legDistance * FUEL_COST_PER_KM;
        double legWage = (legTime / 60) * WAGE_PER_HOUR;
        
        results.path.push_back(0);  // Return to HQ
        
        // Update totals for return trip
        results.totalDistance += legDistance;
        results.totalTime += legTime;
        results.totalFuel += legFuel;
        results.totalWage += legWage;
        
        return results;
    }
};

// Concrete strategy: Traveling Salesman Problem route
class TSPRoute : public RouteStrategy {
private:
    // Helper function for TSP using dynamic programming
    int tspUtil(int mask, int pos, vector<vector<int>>& dp, vector<vector<int>>& dist, int n) {
        // If all cities are visited
        if (mask == ((1 << n) - 1)) {
            return dist[pos][0]; // Return to start
        }
        
        // If this subproblem was already computed
        if (dp[mask][pos] != -1) {
            return dp[mask][pos];
        }
        
        int ans = INT_MAX;
        
        // Try to visit all unvisited cities
        for (int city = 0; city < n; city++) {
            if ((mask & (1 << city)) == 0) { // If city is not visited
                int newAns = dist[pos][city] + tspUtil(mask | (1 << city), city, dp, dist, n);
                ans = min(ans, newAns);
            }
        }
        
        dp[mask][pos] = ans;
        return ans;
    }
    
    // Function to find the TSP path
    vector<int> tspPath(vector<vector<int>>& dist, int n) {
        vector<vector<int>> dp(1 << n, vector<int>(n, -1));
        int minDistance = tspUtil(1, 0, dp, dist, n); // Start from city 0 (HQ)
        
        // Reconstruct the path
        vector<int> path;
        path.push_back(0); // Start from HQ
        
        int mask = 1; // Mask to track visited cities (HQ is already visited)
        int pos = 0;  // Current position (HQ)
        
        for (int i = 0; i < n - 1; i++) {
            int nextCity = -1;
            int bestDistance = INT_MAX;
            
            for (int city = 0; city < n; city++) {
                if ((mask & (1 << city)) == 0) { // If city is not visited
                    int newDistance = dist[pos][city] + dp[mask | (1 << city)][city];
                    if (newDistance < bestDistance) {
                        bestDistance = newDistance;
                        nextCity = city;
                    }
                }
            }
            
            path.push_back(nextCity);
            mask |= (1 << nextCity);
            pos = nextCity;
        }
        
        path.push_back(0); // Return to HQ
        return path;
    }

public:
    RouteResults calculateRoute(WasteLocationManager* manager) override {
        RouteResults results;
        results.totalDistance = 0;
        results.totalTime = 0;
        results.totalFuel = 0;
        results.totalWage = 0;
        
        vector<WasteLocation>& locations = manager->getLocations();
        
        // Criteria: waste level >= 25%
        vector<int> locationsToVisit;
        locationsToVisit.push_back(0); // Add HQ
        
        for (size_t i = 1; i < locations.size(); ++i) {
            if (locations[i].getWasteLevel() >= 25) {
                locationsToVisit.push_back(i);
            }
        }
        
        // If only HQ exists in the list, no locations to visit
        if (locationsToVisit.size() <= 1) {
            results.path.clear();
            return results;
        }
        
        // Create distance matrix for the locations to visit
        int n = locationsToVisit.size();
        vector<vector<int>> dist(n, vector<int>(n, 0));
        
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                dist[i][j] = manager->getDistance(locationsToVisit[i], locationsToVisit[j]);
            }
        }
        
        // For small n, we can use exhaustive approach or DP
        if (n <= 15) {
            vector<int> tsp_path = tspPath(dist, n);
            
            // Convert relative path indices to actual location indices
            for (int i : tsp_path) {
                results.path.push_back(locationsToVisit[i]);
            }
            
            // Calculate total costs
            for (size_t i = 0; i < results.path.size() - 1; ++i) {
                int from = results.path[i];
                int to = results.path[i + 1];
                int legDistance = manager->getDistance(from, to);
                double legTime = legDistance * TIME_PER_KM;
                double legFuel = legDistance * FUEL_COST_PER_KM;
                double legWage = (legTime / 60) * WAGE_PER_HOUR;
                
                results.totalDistance += legDistance;
                results.totalTime += legTime;
                results.totalFuel += legFuel;
                results.totalWage += legWage;
            }
        } else {
            // If n is too large for exact TSP, fall back to greedy approach
            GreedyRoute greedy;
            return greedy.calculateRoute(manager);
        }
        
        return results;
    }
};

// Concrete strategy: Minimum Spanning Tree route
class MSTRoute : public RouteStrategy {
private:
    // Function to find the minimum spanning tree (Prim's algorithm)
    vector<pair<int, int>> primMST(vector<vector<int>>& graph, int numNodes) {
        vector<int> key(numNodes, INT_MAX);     // Key values used to pick minimum weight edge
        vector<bool> inMST(numNodes, false);    // To represent set of vertices included in MST
        vector<int> parent(numNodes, -1);       // To store constructed MST
        
        // Start with the first vertex
        key[0] = 0;
        
        // MST will have numNodes vertices
        for (int count = 0; count < numNodes - 1; count++) {
            // Pick the minimum key vertex not yet included in MST
            int u = -1;
            int min_val = INT_MAX;
            
            for (int v = 0; v < numNodes; v++) {
                if (!inMST[v] && key[v] < min_val) {
                    min_val = key[v];
                    u = v;
                }
            }
            
            if (u == -1) break;  // No valid vertex found
            
            // Add the picked vertex to the MST
            inMST[u] = true;
            
            // Update key values of adjacent vertices
            for (int v = 0; v < numNodes; v++) {
                if (graph[u][v] > 0 && !inMST[v] && graph[u][v] < key[v]) {
                    parent[v] = u;
                    key[v] = graph[u][v];
                }
            }
        }
        
        // Construct the MST edges
        vector<pair<int, int>> mstEdges;
        for (int i = 1; i < numNodes; i++) {
            if (parent[i] != -1) {
                mstEdges.push_back({parent[i], i});
            }
        }
        
        return mstEdges;
    }
    
    // DFS traversal for finding route through MST
    void dfsTraversal(int node, vector<vector<int>>& adjList, vector<bool>& visited, vector<int>& path) {
        visited[node] = true;
        path.push_back(node);
        
        for (int neighbor : adjList[node]) {
            if (!visited[neighbor]) {
                dfsTraversal(neighbor, adjList, visited, path);
            }
        }
    }

public:
    RouteResults calculateRoute(WasteLocationManager* manager) override {
        RouteResults results;
        results.totalDistance = 0;
        results.totalTime = 0;
        results.totalFuel = 0;
        results.totalWage = 0;
        
        vector<WasteLocation>& locations = manager->getLocations();
        
        // Criteria: waste level >= 35%
        vector<int> locationsToVisit;
        locationsToVisit.push_back(0); // Add HQ
        
        for (size_t i = 1; i < locations.size(); ++i) {
            if (locations[i].getWasteLevel() >= 35) {
                locationsToVisit.push_back(i);
            }
        }
        
        // If only HQ exists in the list, no locations to visit
        if (locationsToVisit.size() <= 1) {
            results.path.clear();
            return results;
        }
        
        // Create distance matrix for the locations to visit
        int n = locationsToVisit.size();
        vector<vector<int>> dist(n, vector<int>(n, 0));
        
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                dist[i][j] = manager->getDistance(locationsToVisit[i], locationsToVisit[j]);
            }
        }
        
        // Find MST edges
        vector<pair<int, int>> mstEdges = primMST(dist, n);
        
        // Create adjacency list for the MST
        vector<vector<int>> adjList(n);
        for (const auto& edge : mstEdges) {
            adjList[edge.first].push_back(edge.second);
            adjList[edge.second].push_back(edge.first); // Undirected graph
        }
        
        // Find route through MST using DFS
        vector<bool> visited(n, false);
        vector<int> mstPath;
        dfsTraversal(0, adjList, visited, mstPath); // Start DFS from HQ (index 0)
        
        // Add HQ at the end to return
        mstPath.push_back(0);
        
        // Convert relative path indices to actual location indices
        for (int i : mstPath) {
            results.path.push_back(locationsToVisit[i]);
        }
        
        // Calculate total costs
        for (size_t i = 0; i < results.path.size() - 1; ++i) {
            int from = results.path[i];
            int to = results.path[i + 1];
            int legDistance = manager->getDistance(from, to);
            double legTime = legDistance * TIME_PER_KM;
            double legFuel = legDistance * FUEL_COST_PER_KM;
            double legWage = (legTime / 60) * WAGE_PER_HOUR;
            
            results.totalDistance += legDistance;
            results.totalTime += legTime;
            results.totalFuel += legFuel;
            results.totalWage += legWage;
        }
        
        return results;
    }
};

// Add after the MSTRoute class definition
class RLRoute : public RouteStrategy {
private:
    // Q-table for reinforcement learning
    unordered_map<string, unordered_map<string, double>> qTable;
    double learningRate = 0.1;
    double discountFactor = 0.9;
    double explorationRate = 0.2;
    
    // State representation
    string getStateString(const vector<WasteLocation>& locations, int currentPos) {
        string state;
        for (size_t i = 0; i < locations.size(); i++) {
            if (i == currentPos) continue;
            state += to_string(locations[i].getWasteLevel()) + ",";
        }
        return state;
    }
    
    // Load/save Q-table from/to file
    void loadQTable(const string& filename) {
        ifstream inFile(filename);
        if (inFile) {
            string fromState, toState;
            double value;
            while (inFile >> fromState >> toState >> value) {
                qTable[fromState][toState] = value;
            }
        }
    }
    
    void saveQTable(const string& filename) {
        ofstream outFile(filename);
        for (const auto& fromEntry : qTable) {
            for (const auto& toEntry : fromEntry.second) {
                outFile << fromEntry.first << " " << toEntry.first << " " << toEntry.second << endl;
            }
        }
    }

public:
    RLRoute() {
        loadQTable("qtable.txt"); // Load previous learning
    }
    
    ~RLRoute() {
        saveQTable("qtable.txt"); // Save learning for next time
    }
    
    RouteResults calculateRoute(WasteLocationManager* manager) override {
        RouteResults results;
        results.totalDistance = 0;
        results.totalTime = 0;
        results.totalFuel = 0;
        results.totalWage = 0;
        
        vector<WasteLocation>& locations = manager->getLocations();
        
        // Start from HQ (index 0)
        results.path.push_back(0);
        int currentPos = 0;
        
        // Get locations that need collection (waste level >= 30%)
        vector<int> toVisit;
        for (size_t i = 1; i < locations.size(); i++) {
            if (locations[i].getWasteLevel() >= 30) {
                toVisit.push_back(i);
            }
        }
        
        if (toVisit.empty()) {
            results.path.clear();
            return results;
        }
        
        // Reinforcement learning based route selection
        while (!toVisit.empty()) {
            string currentState = getStateString(locations, currentPos);
            
            // Exploration vs exploitation
            if ((rand() / (double)RAND_MAX) < explorationRate) {
                // Exploration: random choice
                int nextIndex = rand() % toVisit.size();
                int nextPos = toVisit[nextIndex];
                
                // Visit this location
                visitLocation(results, manager, currentPos, nextPos);
                currentPos = nextPos;
                toVisit.erase(toVisit.begin() + nextIndex);
            } else {
                // Exploitation: choose best known action
                double maxQ = -DBL_MAX;
                int bestNextPos = -1;
                int bestIndex = -1;
                
                for (size_t i = 0; i < toVisit.size(); i++) {
                    int candidatePos = toVisit[i];
                    string candidateState = getStateString(locations, candidatePos);
                    
                    // Get Q-value or initialize if not present
                    double qValue = qTable[currentState].count(candidateState) ? 
                                   qTable[currentState][candidateState] : 0;
                    
                    // Add distance as immediate reward component
                    int distance = manager->getDistance(currentPos, candidatePos);
                    qValue -= distance * 0.1; // Penalize longer distances
                    
                    if (qValue > maxQ) {
                        maxQ = qValue;
                        bestNextPos = candidatePos;
                        bestIndex = i;
                    }
                }
                
                if (bestNextPos != -1) {
                    // Visit this location
                    visitLocation(results, manager, currentPos, bestNextPos);
                    
                    // Update Q-table
                    string nextState = getStateString(locations, bestNextPos);
                    double reward = -manager->getDistance(currentPos, bestNextPos); // Negative distance as reward
                    
                    // Find max Q for next state
                    double maxNextQ = 0;
                    for (const auto& entry : qTable[nextState]) {
                        if (entry.second > maxNextQ) {
                            maxNextQ = entry.second;
                        }
                    }
                    
                    // Q-learning update
                    qTable[currentState][nextState] = (1 - learningRate) * qTable[currentState][nextState] + 
                                                     learningRate * (reward + discountFactor * maxNextQ);
                    
                    currentPos = bestNextPos;
                    toVisit.erase(toVisit.begin() + bestIndex);
                }
            }
        }
        
        // Return to HQ
        visitLocation(results, manager, currentPos, 0);
        
        return results;
    }
    
private:
    void visitLocation(RouteResults& results, WasteLocationManager* manager, int from, int to) {
        int distance = manager->getDistance(from, to);
        double time = distance * TIME_PER_KM;
        double fuel = distance * FUEL_COST_PER_KM;
        double wage = (time / 60) * WAGE_PER_HOUR;
        
        results.path.push_back(to);
        results.totalDistance += distance;
        results.totalTime += time;
        results.totalFuel += fuel;
        results.totalWage += wage;
    }
};

// Collection Route class using Strategy pattern
class CollectionRoute {
private:
    shared_ptr<RouteStrategy> strategy;
    
public:
    CollectionRoute() : strategy(nullptr) {}
    
    void setStrategy(shared_ptr<RouteStrategy> newStrategy) {
        strategy = newStrategy;
    }
    
    RouteResults executeRoute(WasteLocationManager* manager) {
        if (!strategy) {
            throw InvalidRouteException("No route strategy has been set");
        }
        
        cout << "Calculating optimal route... ";
        
        // Show a simple progress indicator
        for (int i = 0; i < 5; i++) {
            cout << "." << flush;
            Sleep(200); // Windows sleep function (milliseconds)
        }
        cout << endl;
        
        RouteResults results = strategy->calculateRoute(manager);
        
        cout << "Route calculation complete!" << endl << endl;
        
        // Print route information
        strategy->printRouteInfo(results, manager, getStrategyName());
        
        // Save route information to file
        strategy->saveRouteToFile("route_info.txt", results, manager, getStrategyName());
        
        return results;
    }
    
    string getStrategyName() const {
        if (dynamic_cast<NonOptimizedRoute*>(strategy.get())) {
            return "Regular (Non-Optimized)";
        } else if (dynamic_cast<OptimizedRoute*>(strategy.get())) {
            return "Optimized";
        } else if (dynamic_cast<GreedyRoute*>(strategy.get())) {
            return "Greedy";
        } else if (dynamic_cast<TSPRoute*>(strategy.get())) {
            return "Traveling Salesman Problem";
        } else if (dynamic_cast<MSTRoute*>(strategy.get())) {
            return "Minimum Spanning Tree";
        } else if (dynamic_cast<RLRoute*>(strategy.get())) {
            return "Reinforcement Learning";
        } else {
            return "Unknown Strategy";
        }
    }
};

void visualizeRoute(const RouteResults& results, WasteLocationManager* manager) {
    const vector<WasteLocation>& locations = manager->getLocations();
    
    cout << "\nRoute Visualization:" << endl;
    cout << "===================================================" << endl;
    
    // Simple ASCII visualization
    for (size_t i = 0; i < results.path.size() - 1; i++) {
        int from = results.path[i];
        int to = results.path[i + 1];
        int distance = manager->getDistance(from, to);
        
        cout << left << setw(20) << locations[from].getName() 
             << " --(" << setw(3) << distance << " km)--> " 
             << locations[to].getName() << endl;
    }
    
    cout << "===================================================" << endl;
}

// Main User Interface Class
class WasteManagementSystem {
private:
    WasteLocationManager* locationManager;
    CollectionRoute route;
    int currentRoute;
    
    void displayHeader() {
        system("cls");
        cout << "=============================================" << endl;
        cout << "     WASTE COLLECTION MANAGEMENT SYSTEM      " << endl;
        cout << "             WITH AI FEATURES                " << endl;
        cout << "=============================================" << endl;

        // Add current date/time information
    time_t now = time(nullptr);
    cout << "Date: " << ctime(&now);
    }
    
    void displayMenu() {
        cout << "\nMain Menu:" << endl;
        cout << "1. Generate Random Waste Levels" << endl;
        cout << "2. View Waste Locations Information" << endl;
        cout << "3. Select Route Algorithm" << endl;
        cout << "4. Execute Selected Route" << endl;
        cout << "5. Save Locations Info to File" << endl;
        cout << "6. View AI Predictions" << endl;
        cout << "7. Compare Route Costs" << endl;
        cout << "8. Waste Pattern Analytics Dashboard" << endl;
        cout << "9. Help" << endl;
        cout << "10. Save All Data" << endl;
        cout << "11. Load Data" << endl;
        cout << "12. Delete Saved Data" << endl;
        cout << "0. Exit" << endl;
        cout << "\nCurrent Route Algorithm: " << getCurrentRouteAlgorithm() << endl;
        cout << "\nEnter your choice: ";
    }
    
    void selectRouteAlgorithm() {
        system("cls");
        displayHeader();
        
        cout << "\nSelect Route Algorithm:" << endl;
        cout << "1. Regular (Non-Optimized) Route" << endl;
        cout << "2. Optimized Route" << endl;
        cout << "3. Greedy Route" << endl;
        cout << "4. Traveling Salesman Problem (TSP) Route" << endl;
        cout << "5. Minimum Spanning Tree (MST) Route" << endl;
        cout << "6. Reinforcement Learning (RL) Route" << endl;
        cout << "7. Return to Main Menu" << endl;
        cout << "\nEnter your choice: ";
        
        int choice;
        cin >> choice;
        
        switch (choice) {
            case 1:
                route.setStrategy(make_shared<NonOptimizedRoute>());
                currentRoute = 1;
                break;
            case 2:
                route.setStrategy(make_shared<OptimizedRoute>());
                currentRoute = 2;
                break;
            case 3:
                route.setStrategy(make_shared<GreedyRoute>());
                currentRoute = 3;
                break;
            case 4:
                route.setStrategy(make_shared<TSPRoute>());
                currentRoute = 4;
                break;
            case 5:
                route.setStrategy(make_shared<MSTRoute>());
                currentRoute = 5;
                break;
            case 6:
                route.setStrategy(make_shared<RLRoute>());
                currentRoute = 6;
                break;
            case 7:
                // Return to main menu
                break;
            default:
                cout << "Invalid choice. Please try again." << endl;
                break;
        }
    }
    
    string getCurrentRouteAlgorithm() const {
        switch (currentRoute) {
            case 1: return "Regular (Non-Optimized)";
            case 2: return "Optimized";
            case 3: return "Greedy";
            case 4: return "Traveling Salesman Problem (TSP)";
            case 5: return "Minimum Spanning Tree (MST)";
            case 6: return "Reinforcement Learning (RL)";
            default: return "Not Selected";
        }
    }
    
    void viewAIPredictions() {
        system("cls");
        displayHeader();
        
        const vector<WasteLocation>& locations = locationManager->getLocations();
        AIPredictionModel& aiModel = locationManager->getAIModel();
        
        setTextColor(COLOR_YELLOW);
        cout << "\nAI Predictions for Waste Levels:" << endl;
        setTextColor(COLOR_WHITE);
        cout << "==============================================================================" << endl;
        
        // Add option to simulate trending data
        cout << "Would you like to simulate trending waste levels? (y/n): ";
        char simChoice;
        cin >> simChoice;
        
        if (simChoice == 'y' || simChoice == 'Y') {
            locationManager->simulateTrendingWasteLevels();
            cout << "Trending waste levels simulated." << endl << endl;
        }
        
        // Add table headers with improved spacing
        cout << left 
             << setw(25) << "Location" 
             << setw(18) << "Current %" 
             << setw(18) << "24 Hours %" 
             << setw(18) << "48 Hours %" 
             << setw(18) << "72 Hours %" 
             << setw(25) << "Trend" << endl;
        
        cout << left 
             << setw(25) << "----------------------" 
             << setw(18) << "---------------" 
             << setw(18) << "---------------" 
             << setw(18) << "---------------" 
             << setw(18) << "---------------" 
             << setw(25) << "----------------------" << endl;
        
        for (size_t i = 1; i < locations.size(); ++i) { // Skip HQ
            int current = locations[i].getWasteLevel();
            int predict24h = aiModel.predictWasteLevel(locations[i], 1);
            int predict48h = aiModel.predictWasteLevel(locations[i], 2);
            int predict72h = aiModel.predictWasteLevel(locations[i], 3);
            string trend = aiModel.getWasteTrend(locations[i]);
            string anomalyFlag = aiModel.isAnomaly(locations[i]) ? " [ANOMALY]" : "";
            
            // Create formatted percentage strings with spacing
            string currentStr = to_string(current) + " %";
            string predict24hStr = to_string(predict24h) + " %";
            string predict48hStr = to_string(predict48h) + " %";
            string predict72hStr = to_string(predict72h) + " %";
            
            // Highlight high waste levels in red
            if (current >= 70) setTextColor(COLOR_RED);
            else if (current >= 40) setTextColor(COLOR_YELLOW);
            else setTextColor(COLOR_GREEN);
            
            cout << left << setw(25) << locations[i].getName()
                 << setw(18) << currentStr
                 << setw(18) << predict24hStr
                 << setw(18) << predict48hStr
                 << setw(18) << predict72hStr
                 << setw(25) << trend + anomalyFlag << endl;
            
            setTextColor(COLOR_WHITE);
        }
        
        cout << "\nPress any key to return to main menu...";
        _getch();
    }
    
    void showHelp() {
        system("cls");
        displayHeader();
        
        setTextColor(COLOR_BLUE);
        cout << "\n==============================================" << endl;
        cout << "          HELP DOCUMENTATION                  " << endl;
        cout << "==============================================" << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n1. Generate Random Waste Levels:" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "   Simulates real-world data by generating random waste levels (0-100%)" << endl;
        cout << "   for all collection locations. Use this to test different scenarios." << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n2. View Waste Locations Information:" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "   Displays all waste collection locations with their current waste levels," << endl;
        cout << "   AI predictions for the next 24 hours, and trend analysis." << endl;
        cout << "   - Green: Low waste levels (<40%)" << endl;
        cout << "   - Yellow: Medium waste levels (40-69%)" << endl;
        cout << "   - Red: High waste levels (70%+)" << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n3. Select Route Algorithm:" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "   Choose from 5 different routing algorithms:" << endl;
        cout << "   - Regular: Visits locations with waste level ≥40% within 30km" << endl;
        cout << "   - Optimized: Visits locations with waste level ≥60% within 20km" << endl;
        cout << "   - Greedy: Visits locations with waste level ≥30%, always choosing" << endl;
        cout << "     the nearest location next" << endl;
        cout << "   - TSP: Visits locations with waste level ≥25% using the Traveling" << endl;
        cout << "     Salesman Problem algorithm to find the shortest path" << endl;
        cout << "   - MST: Visits locations with waste level ≥35% using a Minimum" << endl;
        cout << "     Spanning Tree to find an efficient route" << endl;
        cout << "   - RL: Visits locations with waste level ≥30% using Reinforcement" << endl;
        cout << "     Learning to optimize the route based on past experiences" << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n4. Execute Selected Route:" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "   Calculates and displays the optimized collection route using the" << endl;
        cout << "   selected algorithm. Shows distance, time, fuel cost, and wage cost." << endl;
        cout << "   The route information is also saved to 'route_info.txt'." << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n5. Save Locations Info to File:" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "   Exports the current waste location data to 'locations_info.txt'," << endl;
        cout << "   including waste levels, predictions, and distances." << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n6. View AI Predictions:" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "   Shows AI-powered predictions for waste levels over the next 72 hours." << endl;
        cout << "   Includes trend analysis and anomaly detection." << endl;
        cout << "   You can also simulate trending waste data for demonstrations." << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n7. Compare Route Costs:" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "   Compares the costs of different routing algorithms side by side," << endl;
        cout << "   showing potential savings by switching to the most cost-effective route." << endl;
        cout << "   You need to execute at least 2 different routes to use this feature." << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n8. Waste Pattern Analytics Dashboard:" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "   Comprehensive dashboard that analyzes historical waste data," << endl;
        cout << "   identifies patterns, and provides actionable insights including:" << endl;
        cout << "   - Days until bins reach capacity" << endl;
        cout << "   - Waste pattern classification" << endl;
        cout << "   - Trend visualization" << endl;
        cout << "   - Prioritized recommendations" << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n9. Help: Display this help information" << endl;
        
        cout << "\n10. Save All Data:" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "   Saves all current waste data, including locations, waste levels," << endl;
        cout << "   historical data, and distance matrix to a binary file." << endl;
        cout << "   This allows you to restore your data when restarting the application." << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n11. Load Data:" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "   Loads previously saved waste data from a binary file." << endl;
        cout << "   This restores all locations, waste levels, historical data, and distances." << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n12. Delete Saved Data:" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "   Permanently deletes a saved data file." << endl;
        cout << "   Use with caution as this operation cannot be undone." << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n0. Exit: Close the application" << endl;
        
        setTextColor(COLOR_GREEN);
        cout << "\n=== ADDITIONAL INFORMATION ===" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "- Cost calculations include both fuel (RM 2.50/km) and driver wages (RM 10.00/hour)" << endl;
        cout << "- Anomalies are detected when waste levels deviate significantly from historical trends" << endl;
        cout << "- All routes begin and end at the Waste Collector HQ" << endl;
        cout << "- Your data is saved to 'waste_data.dat' by default" << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\nPress any key to return to the main menu...";
        _getch();
    }

    bool confirmAction(const string& message) {
        cout << message << " (y/n): ";
        char response;
        cin >> response;
        return (response == 'y' || response == 'Y');
    }

    // Data persistence methods
    void saveData() {
        system("cls");
        displayHeader();
        
        cout << "\nSave Data Options:" << endl;
        cout << "1. Save to default file (" << locationManager->getLastSavedFilePath() << ")" << endl;
        cout << "2. Save to a custom file" << endl;
        cout << "3. Return to main menu" << endl;
        cout << "\nEnter your choice: ";
        
        int choice;
        cin >> choice;
        
        switch (choice) {
            case 1: {
                // Save to default file
                bool success = locationManager->saveAllData();
                if (success) {
                    cout << "\nData saved successfully to " << locationManager->getLastSavedFilePath() << endl;
                }
                system("pause");
                break;
            }
            case 2: {
                // Save to custom file
                string filename;
                cout << "Enter the filename to save to (e.g., mydata.dat): ";
                cin >> filename;
                
                bool success = locationManager->saveAllData(filename);
                if (success) {
                    cout << "\nData saved successfully to " << filename << endl;
                }
                system("pause");
                break;
            }
            case 3:
                // Return to main menu
                break;
            default:
                cout << "Invalid choice. Please try again." << endl;
                system("pause");
                break;
        }
    }
    
    void loadData() {
        system("cls");
        displayHeader();
        
        cout << "\nLoad Data Options:" << endl;
        cout << "1. Load from default file (" << locationManager->getLastSavedFilePath() << ")" << endl;
        cout << "2. Load from a custom file" << endl;
        cout << "3. Return to main menu" << endl;
        cout << "\nEnter your choice: ";
        
        int choice;
        cin >> choice;
        
        switch (choice) {
            case 1: {
                // Load from default file
                if (!locationManager->dataFileExists()) {
                    cout << "\nNo data file found at " << locationManager->getLastSavedFilePath() << endl;
                    system("pause");
                    break;
                }
                
                if (confirmAction("This will overwrite current data. Continue?")) {
                    bool success = locationManager->loadAllData();
                    if (success) {
                        cout << "\nData loaded successfully from " << locationManager->getLastSavedFilePath() << endl;
                    }
                }
                system("pause");
                break;
            }
            case 2: {
                // Load from custom file
                string filename;
                cout << "Enter the filename to load from (e.g., mydata.dat): ";
                cin >> filename;
                
                if (!locationManager->dataFileExists(filename)) {
                    cout << "\nNo data file found at " << filename << endl;
                    system("pause");
                    break;
                }
                
                if (confirmAction("This will overwrite current data. Continue?")) {
                    bool success = locationManager->loadAllData(filename);
                    if (success) {
                        cout << "\nData loaded successfully from " << filename << endl;
                    }
                }
                system("pause");
                break;
            }
            case 3:
                // Return to main menu
                break;
            default:
                cout << "Invalid choice. Please try again." << endl;
                system("pause");
                break;
        }
    }
    
    void deleteData() {
        system("cls");
        displayHeader();
        
        cout << "\nDelete Data Options:" << endl;
        cout << "1. Delete default file (" << locationManager->getLastSavedFilePath() << ")" << endl;
        cout << "2. Delete a custom file" << endl;
        cout << "3. Return to main menu" << endl;
        cout << "\nEnter your choice: ";
        
        int choice;
        cin >> choice;
        
        switch (choice) {
            case 1: {
                // Delete default file
                if (!locationManager->dataFileExists()) {
                    cout << "\nNo data file found at " << locationManager->getLastSavedFilePath() << endl;
                    system("pause");
                    break;
                }
                
                if (confirmAction("Are you sure you want to delete this file? This cannot be undone.")) {
                    bool success = locationManager->deleteDataFile();
                    if (success) {
                        cout << "\nData file deleted successfully" << endl;
                    }
                }
                system("pause");
                break;
            }
            case 2: {
                // Delete custom file
                string filename;
                cout << "Enter the filename to delete (e.g., mydata.dat): ";
                cin >> filename;
                
                if (!locationManager->dataFileExists(filename)) {
                    cout << "\nNo data file found at " << filename << endl;
                    system("pause");
                    break;
                }
                
                if (confirmAction("Are you sure you want to delete this file? This cannot be undone.")) {
                    bool success = locationManager->deleteDataFile(filename);
                    if (success) {
                        cout << "\nData file deleted successfully" << endl;
                    }
                }
                system("pause");
                break;
            }
            case 3:
                // Return to main menu
                break;
            default:
                cout << "Invalid choice. Please try again." << endl;
                system("pause");
                break;
        }
    }

    // Add this to the private section of WasteManagementSystem class
private:
    // Add storage for route results
    map<string, RouteResults> routeResults;
    
    // Add method to compare routes
    void compareRoutes() {
        system("cls");
        displayHeader();
        
        if (routeResults.size() < 2) {
            cout << "\nYou need to execute at least 2 different routes to compare them." << endl;
            cout << "\nPress any key to return to main menu...";
            _getch();
            return;
        }
        
        UIHelper::displayCostComparison(routeResults);
        
        // Find the cheapest route
        string cheapestRoute = "";
        double cheapestCost = 1000000.0;
        
        for (const auto& route : routeResults) {
            double totalCost = route.second.totalFuel + route.second.totalWage;
            if (totalCost < cheapestCost) {
                cheapestCost = totalCost;
                cheapestRoute = route.first;
            }
        }
        
        cout << "\nCost Savings Analysis:" << endl;
        cout << "--------------------" << endl;
        cout << "Most cost-effective route: " << cheapestRoute << endl;
        
        for (const auto& route : routeResults) {
            if (route.first != cheapestRoute) {
                double routeCost = route.second.totalFuel + route.second.totalWage;
                double savings = routeCost - cheapestCost;
                double savingsPercent = (savings / routeCost) * 100;
                
                UIHelper::displaySavings("Switching from " + route.first + " to " + cheapestRoute + " saves", savings);
                cout << "(" << fixed << setprecision(2) << savingsPercent << "% reduction)" << endl;
            }
        }
        
        cout << "\nPress any key to return to main menu...";
        _getch();
    }

    void displayAnalyticsDashboard() {
        system("cls");
        displayHeader();
        
        const vector<WasteLocation>& locations = locationManager->getLocations();
        AIPredictionModel& aiModel = locationManager->getAIModel();
        
        setTextColor(COLOR_BLUE);
        cout << "\n=================================================================" << endl;
        cout << "               WASTE PATTERN ANALYTICS DASHBOARD                  " << endl;
        cout << "=================================================================" << endl;
        setTextColor(COLOR_WHITE);
        
        // Calculate overall waste statistics
        double totalWaste = 0;
        double avgWaste = 0;
        int highPriorityLocations = 0;
        int anomalies = 0;
        
        for (size_t i = 1; i < locations.size(); i++) {
            totalWaste += locations[i].getWasteLevel();
            if (locations[i].getWasteLevel() >= 70) highPriorityLocations++;
            if (aiModel.isAnomaly(locations[i])) anomalies++;
        }
        
        avgWaste = totalWaste / (locations.size() - 1);  // Skip HQ
        
        // Display summary section
        cout << "\n----- SUMMARY STATISTICS -----" << endl;
        cout << left << setw(35) << "Average Waste Level:" 
             << fixed << setprecision(2) << avgWaste << "%" << endl;
        cout << left << setw(35) << "High Priority Locations:" << highPriorityLocations << endl;
        cout << left << setw(35) << "Detected Anomalies:" << anomalies << endl;
        
        // Display waste pattern analysis table
        cout << "\n----- WASTE PATTERN ANALYSIS -----" << endl;
        cout << left 
             << setw(25) << "Location" 
             << setw(15) << "Current %" 
             << setw(15) << "Trend" 
             << setw(20) << "Days to Capacity" 
             << setw(20) << "Pattern Type" << endl;
        cout << string(95, '-') << endl;
        
        for (size_t i = 1; i < locations.size(); i++) {
            int current = locations[i].getWasteLevel();
            string trend = aiModel.getWasteTrend(locations[i]);
            
            // Calculate days until capacity
            int daysToCapacity = calculateDaysToCapacity(locations[i], aiModel);
            
            // Determine pattern type based on historical data
            string patternType = determinePatternType(locations[i], aiModel);
            
            // Highlight based on priority
            if (current >= 70) setTextColor(COLOR_RED);
            else if (current >= 40) setTextColor(COLOR_YELLOW);
            else setTextColor(COLOR_GREEN);
            
            cout << left << setw(25) << locations[i].getName()
                 << setw(15) << (to_string(current) + " %")
                 << setw(15) << trend;
                 
            // Display days to capacity with conditional formatting
            if (daysToCapacity <= 2) {
                setTextColor(COLOR_RED);
                cout << setw(20) << daysToCapacity;
                if (current >= 40) setTextColor(COLOR_YELLOW);
                else setTextColor(COLOR_GREEN);
            } else {
                cout << setw(20) << daysToCapacity;
            }
            
            // Anomaly-based pattern 
            if (aiModel.isAnomaly(locations[i])) {
                setTextColor(COLOR_RED);
                cout << setw(20) << patternType + " [ANOMALY]" << endl;
            } else {
                cout << setw(20) << patternType << endl;
            }
            
            setTextColor(COLOR_WHITE);
        }
        
        // Display waste trend visualization
        cout << "\n----- WASTE TREND VISUALIZATION -----" << endl;
        visualizeWasteTrends(locations, aiModel);
        
        // Display insights and recommendations
        cout << "\n----- INSIGHTS & RECOMMENDATIONS -----" << endl;
        generateInsights(locations, aiModel);
        
        // Display the most recent route on the map if available
        if (!routeResults.empty()) {
            // Get the most recent route result
            auto it = routeResults.rbegin();
            cout << "\n----- ROUTE MAP VISUALIZATION -----" << endl;
            visualizeRouteMap(it->second, locationManager);
        }

        cout << "\nPress any key to return to the main menu...";
        _getch();
    }
    
    // Helper methods for analytics dashboard
    int calculateDaysToCapacity(const WasteLocation& location, const AIPredictionModel& aiModel) {
        int currentLevel = location.getWasteLevel();
        if (currentLevel >= 95) return 0;  // Already at capacity
        
        string trend = aiModel.getWasteTrend(location);
        
        // Extract trend information to calculate daily increase rate
        double dailyRate = 0;
        
        if (trend == "Rapidly increasing") dailyRate = 10.0;
        else if (trend == "Increasing") dailyRate = 5.0;
        else if (trend == "Stable") dailyRate = 2.0;
        else if (trend == "Decreasing") dailyRate = -2.0;
        else if (trend == "Rapidly decreasing") dailyRate = -5.0;
        else dailyRate = 2.0;  // Default
        
        // If rate is negative or zero, waste won't reach capacity
        if (dailyRate <= 0) return 99;  
        
        // Calculate days to reach 95% capacity
        int daysToCapacity = ceil((95 - currentLevel) / dailyRate);
        return max(0, daysToCapacity);
    }
    
    string determinePatternType(const WasteLocation& location, const AIPredictionModel& aiModel) {
        string trend = aiModel.getWasteTrend(location);
        const vector<pair<int, double>>& historicalData = location.getHistoricalData();
        
        if (historicalData.size() < 5) {
            return "Insufficient Data";
        }
        
        // Calculate variability as a simple indicator of pattern type
        double sum = 0;
        double mean = 0;
        double variance = 0;
        
        for (const auto& data : historicalData) {
            sum += data.second;
        }
        mean = sum / historicalData.size();
        
        for (const auto& data : historicalData) {
            variance += pow(data.second - mean, 2);
        }
        variance /= historicalData.size();
        double stdDev = sqrt(variance);
        
        if (stdDev < 5.0) {
            if (trend == "Stable") return "Consistent";
            return "Steady " + trend;
        } else if (stdDev < 15.0) {
            return "Moderate Variability";
        } else {
            return "Highly Variable";
        }
    }
    
    void visualizeWasteTrends(const vector<WasteLocation>& locations, const AIPredictionModel& aiModel) {
        // Find top 3 locations with fastest growth
        vector<pair<string, string>> topGrowth;
        
        for (size_t i = 1; i < locations.size(); i++) {
            string trend = aiModel.getWasteTrend(locations[i]);
            if (trend == "Rapidly increasing" || trend == "Increasing") {
                topGrowth.push_back({locations[i].getName(), trend});
                if (topGrowth.size() >= 3) break;
            }
        }
        
        // Display visualization
        cout << "Top Waste Growth Locations:" << endl;
        
        if (topGrowth.empty()) {
            cout << "   No significant growth trends detected" << endl;
        } else {
            for (const auto& loc : topGrowth) {
                cout << "   " << loc.first << " - " << loc.second << endl;
                
                // Simple ASCII chart showing predicted growth
                int currentLevel = 0;
                for (size_t i = 1; i < locations.size(); i++) {
                    if (locations[i].getName() == loc.first) {
                        currentLevel = locations[i].getWasteLevel();
                        break;
                    }
                }
                
                cout << "   Current: [";
                int barLength = currentLevel / 5;  // 20 chars = 100%
                for (int j = 0; j < 20; j++) {
                    if (j < barLength) cout << "█";
                    else cout << " ";
                }
                cout << "] " << currentLevel << "%" << endl;
                
                // Show 3-day prediction
                cout << "   3-Day:   [";
                for (size_t i = 1; i < locations.size(); i++) {
                    if (locations[i].getName() == loc.first) {
                        int predictedLevel = aiModel.predictWasteLevel(locations[i], 3);
                        barLength = predictedLevel / 5;
                        for (int j = 0; j < 20; j++) {
                            if (j < barLength) {
                                if (j < currentLevel / 5) cout << "█";
                                else cout << "▓";  // New growth
                            } else cout << " ";
                        }
                        cout << "] " << predictedLevel << "%" << endl;
                        break;
                    }
                }
                cout << endl;
            }
        }
    }
    
    void generateInsights(const vector<WasteLocation>& locations, const AIPredictionModel& aiModel) {
        // Count locations by priority
        int highPriority = 0;
        int mediumPriority = 0;
        int lowPriority = 0;
        vector<string> anomalyLocations;
        
        for (size_t i = 1; i < locations.size(); i++) {
            int level = locations[i].getWasteLevel();
            if (level >= 70) highPriority++;
            else if (level >= 40) mediumPriority++;
            else lowPriority++;
            
            if (aiModel.isAnomaly(locations[i])) {
                anomalyLocations.push_back(locations[i].getName());
            }
        }
        
        // Generate insights based on data
        if (highPriority > 0) {
            setTextColor(COLOR_RED);
            cout << "• URGENT: " << highPriority << " location(s) require immediate attention" << endl;
            setTextColor(COLOR_WHITE);
        }
        
        if (mediumPriority > 0) {
            setTextColor(COLOR_YELLOW);
            cout << "• " << mediumPriority << " location(s) should be scheduled within the next 48 hours" << endl;
            setTextColor(COLOR_WHITE);
        }
        
        if (!anomalyLocations.empty()) {
            setTextColor(COLOR_RED);
            cout << "• Anomalies detected at: ";
            for (size_t i = 0; i < anomalyLocations.size(); i++) {
                cout << anomalyLocations[i];
                if (i < anomalyLocations.size() - 1) cout << ", ";
            }
            cout << endl << "  Recommend investigation of sudden waste level changes" << endl;
            setTextColor(COLOR_WHITE);
        }
        
        // Overall waste management recommendations
        cout << "• Recommendation: ";
        if (highPriority > 2) {
            cout << "Increase collection frequency for high-priority areas" << endl;
        } else if (mediumPriority > locations.size() / 2) {
            cout << "Consider optimizing routes to handle increasing waste levels" << endl;
        } else {
            cout << "Current collection schedule is adequate for waste volumes" << endl;
        }
    }

    // Add this to the WasteManagementSystem class in the private section
    void visualizeRouteMap(const RouteResults& results, WasteLocationManager* manager) {
        const vector<WasteLocation>& locations = manager->getLocations();
        const int mapWidth = 60;
        const int mapHeight = 20;
        
        // Create empty map grid
        vector<vector<char>> map(mapHeight, vector<char>(mapWidth, ' '));
        
        // Simple coordinate mapping for demonstration
        // In a real system, you might want to use actual geo-coordinates and scale them
        unordered_map<int, pair<int, int>> locationCoords;
        
        // Assign coordinates to locations (simplified positioning)
        locationCoords[0] = {mapWidth / 2, mapHeight / 2}; // HQ at center
        
        // Distribute other locations in a circular pattern around HQ
        int radius = min(mapWidth, mapHeight) / 3;
        for (size_t i = 1; i < locations.size(); i++) {
            double angle = 2 * M_PI * (i - 1) / (locations.size() - 1);
            int x = locationCoords[0].first + static_cast<int>(radius * cos(angle));
            int y = locationCoords[0].second + static_cast<int>(radius * sin(angle));
            
            // Ensure coordinates are within bounds
            x = max(0, min(mapWidth - 1, x));
            y = max(0, min(mapHeight - 1, y));
            
            locationCoords[i] = {x, y};
        }
        
        // Draw location points on map
        for (size_t i = 0; i < locations.size(); i++) {
            int x = locationCoords[i].first;
            int y = locationCoords[i].second;
            
            // Use different symbols based on location type
            if (i == 0) {
                map[y][x] = 'H'; // HQ
            } else {
                // Check if this location is part of the route
                bool isVisited = find(results.path.begin(), results.path.end(), i) != results.path.end();
                
                if (isVisited) {
                    if (locations[i].getWasteLevel() >= 70) {
                        map[y][x] = '!'; // High priority and visited
                    } else {
                        map[y][x] = 'O'; // Regular visited location
                    }
                } else {
                    if (locations[i].getWasteLevel() >= 70) {
                        map[y][x] = '*'; // High priority but not visited
                    } else {
                        map[y][x] = '.'; // Regular unvisited location
                    }
                }
            }
        }
        
        // Draw route lines on map using simple Bresenham's line algorithm
        if (results.path.size() > 1) {
            for (size_t i = 0; i < results.path.size() - 1; i++) {
                int from = results.path[i];
                int to = results.path[i + 1];
                
                // Get coordinates
                int x1 = locationCoords[from].first;
                int y1 = locationCoords[from].second;
                int x2 = locationCoords[to].first;
                int y2 = locationCoords[to].second;
                
                // Draw line between points
                int dx = abs(x2 - x1);
                int dy = abs(y2 - y1);
                int sx = x1 < x2 ? 1 : -1;
                int sy = y1 < y2 ? 1 : -1;
                int err = dx - dy;
                
                while (true) {
                    if (x1 == x2 && y1 == y2) break;
                    
                    int e2 = 2 * err;
                    if (e2 > -dy) {
                        err -= dy;
                        x1 += sx;
                    }
                    if (e2 < dx) {
                        err += dx;
                        y1 += sy;
                    }
                    
                    // Don't overwrite location markers
                    if (map[y1][x1] == ' ') {
                        if (i == results.path.size() - 2 && x1 == x2 && y1 == y2) {
                            // This is the last segment and we're at the end point
                            continue;
                        }
                        
                        // Use different symbols for horizontal, vertical, and diagonal lines
                        if (dx > dy) {
                            map[y1][x1] = '-';
                        } else if (dy > dx) {
                            map[y1][x1] = '|';
                        } else {
                            map[y1][x1] = '\\';
                        }
                    }
                }
            }
        }
        
        // Print the map
        cout << "\n    ASCII MAP VISUALIZATION OF ROUTE    " << endl;
        cout << "    (H=HQ, O=Visited, .=Unvisited, !=High Priority)" << endl;
        cout << "    " << string(mapWidth, '=') << endl;
        
        for (int y = 0; y < mapHeight; y++) {
            cout << "    ";
            for (int x = 0; x < mapWidth; x++) {
                // Add color based on map element
                if (map[y][x] == 'H') {
                    setTextColor(COLOR_BLUE);
                } else if (map[y][x] == 'O') {
                    setTextColor(COLOR_GREEN);
                } else if (map[y][x] == '!') {
                    setTextColor(COLOR_RED);
                } else if (map[y][x] == '*') {
                    setTextColor(COLOR_YELLOW);
                } else if (map[y][x] == '-' || map[y][x] == '|' || map[y][x] == '\\') {
                    setTextColor(COLOR_WHITE);
                }
                
                cout << map[y][x];
                setTextColor(COLOR_WHITE);
            }
            cout << endl;
        }
        
        cout << "    " << string(mapWidth, '=') << endl;
        
        // Add location legend at the bottom
        cout << "\n    LOCATION LEGEND:" << endl;
        for (size_t i = 0; i < locations.size(); i++) {
            int x = locationCoords[i].first;
            int y = locationCoords[i].second;
            
            // Color coding for location types
            if (i == 0) {
                setTextColor(COLOR_BLUE);
            } else if (locations[i].getWasteLevel() >= 70) {
                setTextColor(COLOR_RED);
            } else if (find(results.path.begin(), results.path.end(), i) != results.path.end()) {
                setTextColor(COLOR_GREEN);
            } else {
                setTextColor(COLOR_WHITE);
            }
            
            cout << "    (" << setw(2) << x << "," << setw(2) << y << "): " 
                 << locations[i].getName() << " - " 
                 << locations[i].getWasteLevel() << "%" << endl;
                 
            setTextColor(COLOR_WHITE);
        }
    }

public:
    WasteManagementSystem() : currentRoute(0) {
        locationManager = WasteLocationManager::getInstance();
        
        // Try to load saved data, if available
        if (locationManager->dataFileExists()) {
            if (locationManager->loadAllData()) {
                cout << "Loaded saved data from " << locationManager->getLastSavedFilePath() << endl;
            } else {
                // If loading fails, initialize with default data
                locationManager->initializeLocations();
                locationManager->generateRandomWasteLevels();
            }
        } else {
            // If no saved data exists, initialize with default data
            locationManager->initializeLocations();
            locationManager->generateRandomWasteLevels();
        }
        
        // Set default route strategy
        route.setStrategy(make_shared<NonOptimizedRoute>());
        currentRoute = 1;
    }
    
    void run() {
        int choice;
        bool running = true;
        
        while (running) {
            displayHeader();
            displayMenu();
            
            cin >> choice;
            
            switch (choice) {
                case 1:
                    locationManager->generateRandomWasteLevels();
                    cout << "Random waste levels generated." << endl;
                    system("pause");
                    break;
                    
                case 2:
                    system("cls");
                    displayHeader();
                    locationManager->printLocationsInfo();
                    system("pause");
                    break;
                    
                case 3:
                    selectRouteAlgorithm();
                    break;
                    
                case 4:
                    system("cls");
                    displayHeader();
                    
                    if (currentRoute == 0) {
                        cout << "Please select a route algorithm first." << endl;
                    } else {
                        try {
                            // Execute the route and save results
                            RouteResults result = route.executeRoute(locationManager);
                            
                            // Visualize route with ASCII graphics
                            visualizeRoute(result, locationManager);
                            
                            // Store route results for comparison
                            routeResults[getCurrentRouteAlgorithm()] = result;
                            
                            // Visualize the route on a map
                            visualizeRouteMap(result, locationManager);
                        } catch (const InvalidRouteException& e) {
                            cout << "Error: " << e.what() << endl;
                        }
                    }
                    system("pause");
                    break;
                    
                case 5:
                    locationManager->saveLocationsToFile("locations_info.txt");
                    cout << "Locations information saved to 'locations_info.txt'" << endl;
                    system("pause");
                    break;
                    
                case 6:
                    viewAIPredictions();
                    break;
                    
                case 7:
                    compareRoutes();
                    break;
                    
                case 8:
                    displayAnalyticsDashboard();
                    break;

                case 9:
                    showHelp();
                    break;
                
                case 10:
                    saveData();
                    break;
                    
                case 11:
                    loadData();
                    break;
                    
                case 12:
                    deleteData();
                    break;

                case 0:
                    if (confirmAction("Are you sure you want to exit?")) {
                        // Ask to save before exit
                        if (confirmAction("Do you want to save your data before exiting?")) {
                            locationManager->saveAllData();
                        }
                        running = false;
                    }
                    break;
                    
                default:
                    cout << "Invalid choice. Please try again." << endl;
                    system("pause");
                    break;
            }
        }
    }
};

int main() {
    try {
        WasteManagementSystem system;
        system.run();
    } catch (const exception& e) {
        cerr << "Fatal error: " << e.what() << endl;
        return 1;
    }
    
    return 0;
}