// Standard C++ libraries for various functionalities
#include <iostream>     // Input/output stream operations
#include <fstream>      // File stream operations for data persistence
#include <string>       // String handling
#include <vector>       // Dynamic arrays
#include <ctime>        // Time-related functions for timestamps
#include <cstdlib>      // General utilities
#include <iomanip>      // I/O manipulation for formatting output
#include <algorithm>    // Standard algorithms like find, sort
#include <conio.h>      // Console I/O for non-blocking input (getch)
#include <queue>        // Queue container for BFS algorithms
#include <stack>        // Stack container for DFS traversal
#include <limits>       // Numeric limits for algorithm boundary conditions
#include <map>          // Ordered associative container
#include <unordered_map>// Hash-based map for faster lookups
#include <memory>       // Smart pointers for automatic memory management
#include <cmath>        // Mathematical functions
#include <windows.h>    // Windows-specific API for console colors
#include <cfloat>       // Floating-point constants like DBL_MAX
#include <unordered_set> // Hash-based set for fast membership testing
#include <sstream>     // String stream for formatted output

// Define PI if not already defined by system headers
#ifndef M_PI
#define M_PI 3.14159265358979323846  // Used for circular position calculations in map visualization
#endif

// Console color constants for visual feedback
#define COLOR_RED 12    // For high-priority items and errors
#define COLOR_GREEN 10  // For success messages and normal waste levels
#define COLOR_BLUE 9    // For informational elements
#define COLOR_YELLOW 14 // For warnings and medium-priority items
#define COLOR_WHITE 15  // Default text color

using namespace std;

class WasteLocation;
class RouteStrategy;         // Abstract base class for all routing algorithms
class NonOptimizedRoute;     // Basic route for locations with ≥40% waste within 30km
class OptimizedRoute;        // Optimized route for locations with ≥60% waste within 20km
class GreedyRoute;           // Nearest-neighbor based route for ≥30% waste levels
class TSPRoute;              // Traveling Salesman Problem route for minimal distance
class MSTRoute;              // Minimum Spanning Tree based route for efficient clustering
class RLRoute;               // Reinforcement Learning route with adaptive optimization
class ExternalFactorsRoute;  // AI-based route optimization considering external factors
class CollectionRoute;       // Facade for route execution with strategy injection
class AIPredictionModel;     // Machine learning model for waste trend analysis and forecasting
class WasteLocationManager;  // Singleton manager for location data with persistence support
class ExternalFactorsRoute;  // AI-based route optimization considering external factors

// Complete definition of RouteResults struct
// Container for route calculation outcomes, used for cost analysis,
// visualization, and comparison between different routing strategies
struct RouteResults {
    vector<int> path;         // Ordered sequence of location indices to visit
    int totalDistance;        // Total route distance in kilometers
    double totalTime;         // Total collection time in minutes
    double totalFuel;         // Total fuel cost in Malaysian Ringgit (RM)
    double totalWage;         // Total wage cost for collection crew in RM
};

// Exception classes
class InvalidRouteException : public exception {
    private:
        string message;  // Custom error message to provide context-specific details about route failures
    public:
        // Constructor captures a descriptive error message to help with debugging route calculation problems
        InvalidRouteException(const string& msg) : message(msg) {}
        // Override of std::exception's what() method to return our custom error message
        // noexcept guarantees this won't throw additional exceptions during error handling
        const char* what() const noexcept override {
            return message.c_str();
        }
    };
/**
 * @class WasteManagementException
 * @brief A custom exception class for waste management errors.
 * 
 * This class is derived from the standard exception class and provides a custom implementation for waste management related exceptions.
 * It contains a string message that can be accessed using the what() method.
 * 
 * @var message The error message associated with the exception.
 * 
 * @public
 * @constructor WasteManagementException
 * @param msg The error message to be associated with the exception.
 * 
 * @public
 * @method const char* what()
 * @brief Returns the error message associated with the exception.
 * @return The error message as a const char pointer.
 */
class WasteManagementException : public exception {
protected:
    string message;
public:
    WasteManagementException(const string& msg) : message(msg) {}
    const char* what() const noexcept override { return message.c_str(); }
};
/**
 * @class LocationNotFoundException
 * @brief Exception class for when a location is not found in waste management.
 */

class LocationNotFoundException : public WasteManagementException {
public:
    LocationNotFoundException(const string& location) 
        : WasteManagementException("Location not found: " + location) {}
};
/**
 * @class InvalidWasteLevelException
 * @brief An exception class for representing invalid waste levels in waste management.
 * 
 * This exception is thrown when an invalid waste level is encountered for a specific location.
 * It is derived from the WasteManagementException class.
 */
class InvalidWasteLevelException : public WasteManagementException {
public:
    InvalidWasteLevelException(const string& location, int level) 
        : WasteManagementException("Invalid waste level " + to_string(level) + 
                                 "% for location: " + location) {}
};
/**
 * @brief A custom exception class for file operations.
 * 
 * This class is derived from the WasteManagementException class and is used to handle exceptions related to file operations.
 */
//Class `FileOperationException` is a custom exception class that is derived from the base class `WasteManagementException`. It represents an exception that occurs when a file operation fails. The class has a constructor that takes two parameters: `operation` (representing the type of operation) and `filename` (representing the name of the file). The constructor initializes the base class `WasteManagementException` with a formatted error message. 
class FileOperationException : public WasteManagementException {
public:
    FileOperationException(const string& operation, const string& filename) 
        : WasteManagementException("File " + operation + " failed for: " + filename) {}
};

/**
 * @class WasteLocation
 * 
 * @brief A class that represents a waste location, storing information about its name, waste level, collection status, historical data, and collection history.
 * 
 * The WasteLocation class provides methods to retrieve and modify the waste level, collection status, and historical data of a waste location. It also allows recording and retrieving collection history.
 * 
 * @public
 * 
 * @param name The name of the waste location.
 * @param wasteLevel The current waste level of the location in percentage.
 * @param isCollected The collection status of the location.
 * @param previousWasteLevels A vector of pairs storing historical waste levels for AI prediction.
 * @param collectionHistory A vector of pairs storing timestamps and collected amounts for collection history.
 * 
 * @member getName Gets the name of the waste location.
 * @return The name of the waste location.
 * 
 * @member getWasteLevel Gets the waste level of the waste location.
 * @return The waste level of the waste location.
 * 
 * @member getIsCollected Gets the collection status of the waste location.
 * @return The collection status of the waste location.
 * 
 * @member setWasteLevel Sets the waste level of the waste location.
 * @param level The new waste level to be set.
 * 
 * @member setIsCollected Sets the collection status of the waste location.
 * @param collected The new collection status to be set.
 * 
 * @member getHistoricalData Gets the historical waste level data of the waste location.
 * @return A const reference to the vector of pairs storing the historical waste level data.
 * 
 * @member addHistoricalDataPoint Adds a new historical waste level data point to the waste location.
 * @param timestamp The timestamp of the data point.
 * @param level The waste level of the data point.
 * 
 * @member clearHistoricalData Clears the historical waste level data of the waste location.
 * 
 * @member recordCollection Records a collection with the current timestamp and amount collected.
 * @param amountCollected The amount of waste collected.
 * 
 * @member recordCollection Records a collection with a specific timestamp and amount collected.
 * @param amountCollected The amount of waste collected.
 * @param timestamp The timestamp of the collection.
 * 
 * @member getCollectionHistory Gets the collection history of the waste location.
 * @return A const reference to the vector of pairs storing the collection history.
 * 
 * @member clearCollectionHistory Clears the collection history of the waste location.
 */
class WasteLocation {
private:
    string name;
    int wasteLevel; // In percentage
    bool isCollected;
    vector<pair<int, double>> previousWasteLevels; // Store historical data for AI prediction
    vector<pair<time_t, double>> collectionHistory; // Store timestamps and collected amounts

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

    // Add method to record a collection
    void recordCollection(double amountCollected) {
        time_t now = time(nullptr);
        collectionHistory.push_back(make_pair(now, amountCollected));
    }
    
    // Add overload for when we have a specific timestamp
    void recordCollection(double amountCollected, time_t timestamp) {
        collectionHistory.push_back(make_pair(timestamp, amountCollected));
    }
    
    // Add getter for collection history
    const vector<pair<time_t, double>>& getCollectionHistory() const {
        return collectionHistory;
    }
    
    // Add method to clear collection history
    void clearCollectionHistory() {
        collectionHistory.clear();
    }
};

/**
 * AIPredictionModel class
 * 
 * This class represents a prediction model for waste levels at different locations. It uses a simple linear regression model to make predictions and detect anomalies in waste levels. The class provides methods to predict waste levels for future days, detect anomalies in waste levels, and determine the trend of waste levels based on historical data.
 * 
 * Class Members:
 * - slope: a double representing the slope of the linear regression model
 * - intercept: a double representing the intercept of the linear regression model
 * - meanWasteLevel: a double representing the mean waste level for anomaly detection
 * - stdDevWasteLevel: a double representing the standard deviation of waste levels for anomaly detection
 * 
 * Methods:
 * - calculateRegressionParameters: a private method that calculates the regression parameters based on historical data
 * - AIPredictionModel: a constructor that initializes the member variables
 * - predictWasteLevel: a method that predicts the waste level for a future day based on historical data and the current waste level
 * - isAnomaly: a method that detects anomalies in waste levels based on historical data and the current waste level
 * - getWasteTrend: a method that determines the trend of waste levels based on historical data
 */
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
            
            if ((n * sumX2 - sumX * sumX) != 0) {
                tempSlope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
                
                // *** Add constraint to the slope to make predictions more realistic ***
                // Limit the maximum daily change to between -15% and +15%
                tempSlope = max(-15.0, min(15.0, tempSlope));
            }
            tempIntercept = (sumY - tempSlope * sumX) / n;
            tempMean = sumY / n;
        } else {
            tempIntercept = historicalData.empty() ? 0 : historicalData[0].second;
        }
        
        // Predict waste level for future day using local variables
        int lastDay = historicalData.back().first;
        int futureDay = lastDay + (daysAhead * 86400); // Convert days to seconds
        
        // For more realistic predictions, add diminishing returns for longer forecasts
        double decayFactor = 1.0 / (1.0 + (daysAhead * 0.2)); // Reduces impact over time
        double adjustedSlope = tempSlope * decayFactor;
        
        double predictedWaste = adjustedSlope * (futureDay - lastDay) + location.getWasteLevel();
        
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



/**
 * Sets the text color in the console.
 *
 * @param colorCode The code representing the desired color.
 */// Helper function for text color
void setTextColor(int colorCode) {
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleTextAttribute(hConsole, colorCode);
}
/**
 * @class UIHelper
 * 
 * This class provides static methods for displaying different types of messages and information on the console.
 * 
 * Methods:
 * - displayError(const string& error): Displays an error message in red color.
 * - displaySuccess(const string& message): Displays a success message in green color.
 * - displayWarning(const string& message): Displays a warning message in yellow color.
 * - displaySavings(const string& message, double amount): Displays a message about savings with a specified amount in green color.
 * - displayProgressBar(int progress): Displays a progress bar with the specified progress percentage.
 * - displayCostComparison(const map<string, RouteResults>& routeResults): Displays a cost comparison table for different routes.
 **/
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
        cout << "\nRoute Cost Comparison:" << endl;
        cout << "====================================================================" << endl;
        cout << left << setw(35) << "Route Type" 
             << setw(20) << "Distance (km)" 
             << setw(20) << "Time (h)" 
             << setw(20) << "Fuel (RM)" 
             << setw(20) << "Wages (RM)" 
             << setw(20) << "Total (RM)" << endl;
        cout << string(105, '-') << endl;
        
        for (const auto& route : routeResults) {
            double totalCost = route.second.totalFuel + route.second.totalWage;
            cout << left << setw(35) << route.first
                 << setw(20) << route.second.totalDistance
                 << setw(20) << fixed << setprecision(2) << (route.second.totalTime / 60)
                 << setw(20) << fixed << setprecision(2) << route.second.totalFuel
                 << setw(20) << fixed << setprecision(2) << route.second.totalWage
                 << setw(20) << fixed << setprecision(2) << totalCost << endl;
        }
        
        cout << "====================================================================" << endl;
    }
};

/** 
 * WasteLocationManager class is responsible for managing waste locations, their data, and performing operations on them.
 * It provides functionality to add locations, initialize them, generate random waste levels, get locations, get distances,
 * print location information, save data to a file, load data from a file, delete data file, check if data file exists,
 * and simulate trending waste levels.
 * 
 * Public Methods:
 *   - void addLocation(const string& name): Adds a waste location with the given name.
 *   - void initializeLocations(): Initializes the waste locations with default data.
 *   - void generateRandomWasteLevels(): Generates random waste levels for each location.
 *   - vector<WasteLocation>& getLocations(): Returns the vector of waste locations.
 *   - int getDistance(int from, int to) const: Returns the distance between two waste locations.
 *   - void printLocationsInfo() const: Prints the information of all waste locations.
 *   - bool saveDataToFile(bool saveBinary = false, const string& filename = ""): Saves the data to a file, either in binary or text format.
 *   - bool loadAllData(const string& filename = ""): Loads all data from a binary file.
 *   - bool deleteDataFile(const string& filename = ""): Deletes the specified data file.
 *   - bool dataFileExists(const string& filename = ""): Checks if the specified data file exists.
 *   - string getLastSavedFilePath() const: Returns the path of the last saved data file.
 *   - void setLastSavedFilePath(const string& filePath): Sets the path of the last saved data file.
 *   - AIPredictionModel& getAIModel(): Returns the AI prediction model.
 *   - void simulateTrendingWasteLevels(): Simulates trending waste levels for each location.
 */
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
            system("cls");
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
    
    // Save data to file - merged function for both text and binary formats
    bool saveDataToFile(bool saveBinary = false, const string& filename = "") {
        string saveFile;
        
        try {
            system("cls");
            cout << "\nSaving data..." << endl;
            
            // Determine filename
            if (filename.empty()) {
                if (saveBinary) {
                    saveFile = lastSavedFilePath;
                } else {
                    saveFile = "locations_info.txt";
                }
            } else {
                saveFile = filename;
            }
            
            UIHelper::displayProgressBar(0);
            
            // Binary mode (for complete data) or text mode (for human-readable)
            if (saveBinary) {
                ofstream outFile(saveFile, ios::binary);
                if (!outFile) {
                    throw FileOperationException("open", saveFile);
                }

                cout << "Saving complete data to " << saveFile << "..." << endl;
                
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
                    
                    // Save collection history
            const vector<pair<time_t, double>>& collectionHistory = locations[i].getCollectionHistory();
            int collectionHistorySize = collectionHistory.size();
            outFile.write(reinterpret_cast<char*>(&collectionHistorySize), sizeof(collectionHistorySize));
            
            for (const auto& point : collectionHistory) {
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
                
                // Save the file path for future reference
                lastSavedFilePath = saveFile;
            } else {
                // Text mode for human-readable output
                ofstream outFile(saveFile);
                if (!outFile) {
                    throw FileOperationException("open", saveFile);
                }

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
                outFile << "===========================================\n";
                
                // Header row with location names
                outFile << setw(20) << "From \\ To";
                for (size_t j = 0; j < locations.size(); j++) {
                    outFile << setw(15) << locations[j].getName();
                }
                outFile << endl;
                
                // Matrix data
                for (size_t i = 0; i < locations.size(); i++) {
                    outFile << setw(20) << locations[i].getName();
                    for (size_t j = 0; j < locations.size(); j++) {
                        outFile << setw(15) << distanceMatrix[i][j];
                    }
                    outFile << endl;
                }
                UIHelper::displayProgressBar(90);
            }
            
            UIHelper::displayProgressBar(100);
            cout << endl;
            
            // Success message
            if (saveBinary) {
                UIHelper::displaySuccess("Complete data saved successfully to " + saveFile);
            } else {
                UIHelper::displaySuccess("Location information saved successfully to " + saveFile);
            }
            
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

                // Read collection history
        int collectionHistorySize;
        inFile.read(reinterpret_cast<char*>(&collectionHistorySize), sizeof(collectionHistorySize));
        
        for (int j = 0; j < collectionHistorySize; j++) {
            time_t timestamp;
            double amount;
            inFile.read(reinterpret_cast<char*>(&timestamp), sizeof(timestamp));
            inFile.read(reinterpret_cast<char*>(&amount), sizeof(amount));
            
            location.recordCollection(amount, timestamp);
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
            
            // Stronger trend strength for more visible changes
            double trendStrength = (5.0 + (rand() % 15) / 5.0) * (trendDirection == 0 ? 0.1 : trendDirection);
            
            // Simulate historical data points that will create different predictions for different time frames
            for (int j = 10; j >= 0; j--) {
                // Create a progressively stronger effect for more distant predictions
                // Make trend effect proportional to how far in the future we're predicting
                double timeMultiplier = (10 - j) * 1.5;
                
                // Apply randomization that increases with prediction distance
                double randomFactor = (10 - j) * (rand() % 15) / 10.0;
                
                // Base trend effect with increasing impact over time
                double trendEffect = timeMultiplier * trendStrength;
                
                // Add the random factor to create more variation
                if (rand() % 2 == 0) {
                    trendEffect += randomFactor;
                } else {
                    trendEffect -= randomFactor;
                }
                
                // Calculate level within bounds
                int historicalLevel = max(5, min(95, static_cast<int>(currentLevel - trendEffect)));
                
                // Store historical data point
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

/**
 * @class RouteStrategy
 * 
 * @brief A base class for implementing various route calculation strategies.
 * 
 * This class provides a framework for calculating routes based on different strategies. It contains constants for cost calculation, a pure virtual method to be implemented by concrete strategies, and methods to save route information to a file and print route information to the console.
 * 
 * The RouteStrategy class is designed to be inherited from and extended by concrete strategy classes. It provides the necessary infrastructure for route calculation and output, while allowing for customization and flexibility through the implementation of the calculateRoute method in derived classes.
 * 
 * The class also includes helper methods to save and print route information, which can be used by derived classes to generate route reports in different formats.
 * 
 * @note This class is intended to be used as a base class and should not be instantiated directly.
 */
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

/**
 * @class NonOptimizedRoute
 * 
 * @brief A class that represents a non-optimized route strategy for waste collection.
 * 
 * This class extends the RouteStrategy class and provides a method to calculate a route for waste collection.
 * The route is calculated based on the criteria that the waste level of a location is greater than or equal to 40% and the distance to the location is less than or equal to 30km.
 * The class keeps track of the total distance, total time, total fuel cost, and total wage for the calculated route.
 * If any locations are visited, the class also includes the path to return to the headquarters.
 */
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

/**
 * @class OptimizedRoute
 * @brief A class that represents an optimized route strategy for waste management.
 * 
 * This class extends the RouteStrategy base class and provides an implementation of the calculateRoute method.
 * The calculateRoute method calculates the optimized route based on certain criteria, such as waste level and distance.
 * 
 * @public
 * @inherits RouteStrategy
 */
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

/**
 * @class GreedyRoute
 * 
 * @brief A class that implements the RouteStrategy interface using a greedy algorithm to calculate the route for waste collection.
 * 
 * The GreedyRoute class extends the RouteStrategy class and provides an implementation for the calculateRoute method. 
 * It uses a greedy algorithm to determine the most efficient path to collect waste from waste locations based on a given criteria.
 * 
 * The calculateRoute method takes a WasteLocationManager object as a parameter and returns a RouteResults object that contains 
 * information about the calculated route, such as the total distance, total time, total fuel cost, and total wage. 
 * The method uses the greedy algorithm to visit the waste locations in the order of their proximity to the current position, 
 * while considering the waste level criteria. It starts from the headquarters (index 0) and iteratively selects the nearest 
 * unvisited waste location that meets the waste level criteria. The algorithm continues until all eligible waste locations 
 * have been visited or there are no more waste locations to visit.
 * 
 * The calculateRoute method also calculates the costs for each leg of the route, including the time, fuel, and wage. 
 * It updates the total distance, total time, total fuel, and total wage as it visits each waste location. 
 * Finally, it returns to the headquarters to complete the route, updating the totals for the return trip as well.
 * 
 * The GreedyRoute class provides an efficient approach to waste collection by prioritizing waste locations based on their proximity 
 * and waste level, resulting in reduced travel time, fuel consumption, and labor costs.
 */// Concrete strategy: Greedy route
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

/**
 * TSPRoute class is a subclass of RouteStrategy and implements the Traveling Salesman Problem (TSP) algorithm.
 * It calculates the optimal route to visit waste locations with a waste level of at least 25%.
 * 
 * The TSP algorithm is implemented using dynamic programming to find the minimum distance path.
 * 
 * Public Methods:
 *   - calculateRoute: Calculates the optimal route based on waste locations and their distances.
 * 
 * Private Methods:
 *   - tspUtil: Helper function for TSP using dynamic programming.
 *   - tspPath: Function to find the TSP path based on the calculated distances.
 * 
 * Dependencies:
 *   - RouteStrategy: Superclass for defining route calculation strategies.
 *   - WasteLocationManager: Manages waste locations and provides distance information.
 *   - WasteLocation: Represents a waste location with its waste level.
 * 
 * Constants:
 *   - TIME_PER_KM: Time taken to travel 1 kilometer.
 *   - FUEL_COST_PER_KM: Fuel cost to travel 1 kilometer.
 *   - WAGE_PER_HOUR: Wage per hour for the waste collection.
 * 
 * Member Variables:
 *   - totalDistance: Total distance of the calculated route.
 *   - totalTime: Total time taken to travel the calculated route.
 *   - totalFuel: Total fuel cost for the calculated route.
 *   - totalWage: Total wage cost for the waste collection on the calculated route.
 *   - locationsToVisit: List of waste locations to visit, including the headquarters.
 *   - dist: Distance matrix for the waste locations.
 * 
 * Usage:
 *   TSPRoute tspRoute;
 *   RouteResults results = tspRoute.calculateRoute(wasteLocationManager);
 *   // Access the calculated route and other information from the results object.
 */
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

/**
 * \class MSTRoute
 * \brief A class that implements the MSTRoute route strategy.
 *
 * The MSTRoute class is a derived class of the RouteStrategy base class. It provides an implementation of the calculateRoute method, which calculates the optimal route for waste collection based on the minimum spanning tree (MST) algorithm.
 *
 * \note This class assumes that the WasteLocationManager and WasteLocation classes are defined.
 */
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

/**
 * RLRoute class is a RouteStrategy implementation that uses reinforcement learning
 * to calculate the route for waste collection. It extends the RouteStrategy class.
 * 
 * The class contains the following private members:
 * - qTable: unordered_map to store the Q-values for each state-action pair
 * - learningRate: double to specify the learning rate for updating Q-values
 * - discountFactor: double to specify the discount factor for future rewards
 * - explorationRate: double to specify the rate of exploration vs exploitation
 * 
 * The class provides the following public methods:
 * - RLRoute(): constructor that loads the Q-table from a file
 * - ~RLRoute(): destructor that saves the Q-table to a file
 * - calculateRoute(WasteLocationManager* manager): method to calculate the route based on the Q-table
 * 
 * The class also contains the following private methods:
 * - getStateString(const vector<WasteLocation>& locations, int currentPos): method to get the state representation as a string
 * - loadQTable(const string& filename): method to load the Q-table from a file
 * - saveQTable(const string& filename): method to save the Q-table to a file
 * - visitLocation(RouteResults& results, WasteLocationManager* manager, int from, int to): method to visit a waste location and update the route results
 */// Add after the MSTRoute class definition
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

/**
 * @class ExternalFactorsRoute
 * 
 * @brief A class that implements the RouteStrategy interface using AI to optimize routes based on external factors.
 * 
 * The ExternalFactorsRoute class extends the RouteStrategy class and provides an implementation
 * for calculating routes optimized for external factors such as weather conditions, traffic patterns,
 * time of day, and seasonal variations. This strategy uses machine learning techniques to predict
 * the optimal route considering both waste collection needs and environmental/situational factors.
 * 
 * External factors include:
 * - Weather conditions (rain, snow, extreme temperatures)
 * - Traffic patterns and congestion forecasts
 * - Time of day optimization (rush hours vs off-peak)
 * - Seasonal adjustments (tourist seasons, holidays)
 * - Road conditions and maintenance schedules
 * 
 * The class improves overall efficiency by dynamically adjusting routes based on these factors,
 * resulting in fuel savings, reduced emissions, and improved service reliability.
 */
// Concrete strategy: External Factors AI-based Route
class ExternalFactorsRoute : public RouteStrategy {
    private:
        // Constants for external factor weights
        const double WEATHER_WEIGHT = 0.2;
        const double TRAFFIC_WEIGHT = 0.3;
        const double TIME_OF_DAY_WEIGHT = 0.25;
        const double SEASONAL_WEIGHT = 0.15;
        const double ROAD_CONDITION_WEIGHT = 0.1;
        
        // Weather condition factors (0-1 scale, higher means worse)
        double weatherFactor;
        // Traffic congestion factors (0-1 scale, higher means worse)
        double trafficFactor;
        // Time of day factors (0-1 scale, higher means worse)
        double timeOfDayFactor;
        // Seasonal factors (0-1 scale, higher means worse)
        double seasonalFactor;
        // Road condition factors (0-1 scale, higher means worse)
        double roadConditionFactor;
        
        // Database of historical external factors data
        unordered_map<string, vector<double>> historicalData;
        
        // Neural network weights for location predictions
        vector<vector<double>> neuralNetWeights;
        
        // Cached optimal paths for specific conditions
        unordered_map<string, vector<int>> pathCache;
        
        // ML-based road segment prediction model
        vector<double> segmentPredictions;
        
        // Method to get current weather conditions (simulated)
        double getCurrentWeatherFactor() const {
            // In a real implementation, this would connect to a weather API
            // For demonstration, generate random weather factor or use historical pattern
            time_t now = time(nullptr);
            tm* ltm = localtime(&now);
            
            // Simulate seasonal weather patterns - worse in winter months
            if (ltm->tm_mon >= 10 || ltm->tm_mon <= 2) { // November to March
                return 0.6 + ((rand() % 40) / 100.0); // 0.6-1.0 (worse weather)
            } else if (ltm->tm_mon >= 3 && ltm->tm_mon <= 5) { // April to June
                return 0.3 + ((rand() % 40) / 100.0); // 0.3-0.7 (moderate)
            } else {
                return 0.1 + ((rand() % 30) / 100.0); // 0.1-0.4 (better weather)
            }
        }
        
        // Method to get current traffic conditions (simulated)
        double getCurrentTrafficFactor() const {
            // In a real implementation, this would connect to a traffic API
            // For demonstration, simulate traffic based on time of day
            time_t now = time(nullptr);
            tm* ltm = localtime(&now);
            int hour = ltm->tm_hour;
            
            // Simulate rush hour traffic patterns
            if ((hour >= 7 && hour <= 9) || (hour >= 16 && hour <= 18)) {
                return 0.7 + ((rand() % 30) / 100.0); // 0.7-1.0 (heavy traffic)
            } else if ((hour >= 10 && hour <= 15) || (hour >= 19 && hour <= 21)) {
                return 0.3 + ((rand() % 40) / 100.0); // 0.3-0.7 (moderate traffic)
            } else {
                return 0.1 + ((rand() % 30) / 100.0); // 0.1-0.4 (light traffic)
            }
        }
        
        // Method to get time of day factor
        double getTimeOfDayFactor() const {
            // Optimize for time of day - prefer working during daylight and non-rush hours
            time_t now = time(nullptr);
            tm* ltm = localtime(&now);
            int hour = ltm->tm_hour;
            
            // Avoid very early morning, rush hours, and late night
            if (hour < 5 || hour >= 22) {
                return 0.8; // Late night/early morning (challenging)
            } else if ((hour >= 7 && hour <= 9) || (hour >= 16 && hour <= 18)) {
                return 0.7; // Rush hours (avoid)
            } else if (hour >= 10 && hour <= 15) {
                return 0.2; // Midday (ideal)
            } else {
                return 0.5; // Early evening (acceptable)
            }
        }
        
        // Method to get seasonal factor
        double getSeasonalFactor() const {
            // Account for seasonal variations
            time_t now = time(nullptr);
            tm* ltm = localtime(&now);
            int month = ltm->tm_mon;
            int day = ltm->tm_mday;
            
            // Holiday season (December)
            if (month == 11) {
                return 0.7; // Holiday congestion
            }
            // Summer tourist season
            else if (month >= 5 && month <= 8) {
                return 0.6; // Tourist congestion
            }
            // Special holidays (simplified)
            else if ((month == 0 && day == 1) || // New Year
                    (month == 4 && day >= 29 && day <= 31) || // Labor Day
                    (month == 6 && day == 4) || // Independence Day
                    (month == 10 && (day >= 22 && day <= 28))) { // Thanksgiving
                return 0.8; // Holiday congestion
            } else {
                return 0.3; // Normal conditions
            }
        }
        
        // Method to get road condition factor (simulated)
        double getRoadConditionFactor() const {
            // In a real implementation, this would connect to a road condition API
            // For demonstration, simulate based on weather and season
            double weatherEffect = getCurrentWeatherFactor();
            double seasonEffect = getSeasonalFactor();
            
            // Combine weather and seasonal effects with some randomness
            return (weatherEffect * 0.6) + (seasonEffect * 0.3) + ((rand() % 20) / 100.0);
        }
        
        // Calculate the external factor adjusted distance
        int getAdjustedDistance(int actualDistance, int fromLocation, int toLocation) const {
            // Combine all external factors into a single multiplier
            double combinedFactor = (weatherFactor * WEATHER_WEIGHT) +
                                   (trafficFactor * TRAFFIC_WEIGHT) +
                                   (timeOfDayFactor * TIME_OF_DAY_WEIGHT) +
                                   (seasonalFactor * SEASONAL_WEIGHT) +
                                   (roadConditionFactor * ROAD_CONDITION_WEIGHT);
            
            // Apply neural network prediction model (simplified version)
            double locationEffect = 1.0;
            if (!neuralNetWeights.empty() && fromLocation < neuralNetWeights.size() && 
                toLocation < neuralNetWeights[fromLocation].size()) {
                locationEffect = neuralNetWeights[fromLocation][toLocation];
            }
            
            // Calculate adjusted distance - worse conditions = longer effective distance
            double adjustmentFactor = 1.0 + (combinedFactor * locationEffect);
            return static_cast<int>(actualDistance * adjustmentFactor);
        }
        
        // Initialize the neural network weights for location-specific effects
        void initializeNeuralNetwork(int numLocations) {
            // Create a matrix of weights for each location pair
            neuralNetWeights.resize(numLocations);
            
            for (int i = 0; i < numLocations; i++) {
                neuralNetWeights[i].resize(numLocations);
                for (int j = 0; j < numLocations; j++) {
                    if (i == j) {
                        neuralNetWeights[i][j] = 0.0; // No effect for same location
                    } else {
                        // Initialize with random weights between 0.8 and 1.2
                        // In a real implementation, these would be trained on historical data
                        neuralNetWeights[i][j] = 0.8 + ((rand() % 40) / 100.0);
                    }
                }
            }
        }
        
        // Generate a cache key for current conditions
        string generateConditionKey() const {
            // Create a key based on discretized environmental factors
            string key = to_string(static_cast<int>(weatherFactor * 10)) + "_" +
                        to_string(static_cast<int>(trafficFactor * 10)) + "_" +
                        to_string(static_cast<int>(timeOfDayFactor * 10)) + "_" +
                        to_string(static_cast<int>(seasonalFactor * 10)) + "_" +
                        to_string(static_cast<int>(roadConditionFactor * 10));
            return key;
        }
        
        // Method to apply machine learning optimization to the route
        vector<int> optimizeRoute(const vector<int>& candidateLocations, 
                                  WasteLocationManager* manager, 
                                  const vector<WasteLocation>& locations) {
            // Check if we have a cached result for these conditions
            string conditionKey = generateConditionKey();
            if (pathCache.find(conditionKey) != pathCache.end()) {
                return pathCache[conditionKey];
            }
            
            // Create a distance matrix with adjusted distances based on external factors
            int n = candidateLocations.size();
            vector<vector<int>> adjustedDistMatrix(n, vector<int>(n, 0));
            
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    int actualDistance = manager->getDistance(
                        candidateLocations[i], candidateLocations[j]);
                    
                    adjustedDistMatrix[i][j] = getAdjustedDistance(
                        actualDistance, candidateLocations[i], candidateLocations[j]);
                }
            }
            
            // Apply a hybrid optimization algorithm combining aspects of TSP and greedy approaches
            // with adaptations for external factors
            
            // Start with HQ (index 0)
            vector<int> optimizedPath;
            optimizedPath.push_back(candidateLocations[0]);
            
            // Track visited locations
            vector<bool> visited(n, false);
            visited[0] = true;
            
            int currentPos = 0; // Start at first location (HQ)
            int remainingLocations = n - 1;
            
            // Apply a greedy approach with look-ahead and consideration of waste levels
            while (remainingLocations > 0) {
                int bestNextLocation = -1;
                double bestScore = -1.0;
                
                // Evaluate each unvisited location
                for (int i = 1; i < n; i++) {
                    if (!visited[i]) {
                        int locationIndex = candidateLocations[i];
                        
                        // Calculate a score based on:
                        // 1. Adjusted distance (lower is better)
                        // 2. Waste level (higher is better)
                        // 3. Look-ahead potential (evaluate potential next stops)
                        
                        double distanceScore = 1000.0 / (1.0 + adjustedDistMatrix[currentPos][i]);
                        double wasteScore = locations[locationIndex].getWasteLevel() * 10;
                        
                        // Look-ahead: evaluate potential next locations after this one
                        double lookAheadScore = 0.0;
                        int lookAheadCount = 0;
                        
                        for (int j = 1; j < n; j++) {
                            if (!visited[j] && j != i) {
                                // Add a small score component based on proximity of next potential locations
                                lookAheadScore += 500.0 / (1.0 + adjustedDistMatrix[i][j]);
                                lookAheadCount++;
                                
                                // Limit look-ahead to 3 locations for performance
                                if (lookAheadCount >= 3) break;
                            }
                        }
                        
                        // Normalize look-ahead score
                        if (lookAheadCount > 0) {
                            lookAheadScore /= lookAheadCount;
                        }
                        
                        // Combined score with weights
                        double score = (distanceScore * 0.5) + (wasteScore * 0.3) + (lookAheadScore * 0.2);
                        
                        // Update best location if this is better
                        if (score > bestScore) {
                            bestScore = score;
                            bestNextLocation = i;
                        }
                    }
                }
                
                // Add the best location to the path
                if (bestNextLocation != -1) {
                    optimizedPath.push_back(candidateLocations[bestNextLocation]);
                    visited[bestNextLocation] = true;
                    currentPos = bestNextLocation;
                    remainingLocations--;
                } else {
                    // Fallback - shouldn't happen with complete graph
                    break;
                }
            }
            
            // Return to HQ
            optimizedPath.push_back(candidateLocations[0]);
            
            // Cache the result
            pathCache[conditionKey] = optimizedPath;
            
            return optimizedPath;
        }
        
        // Save and report environmental impact information
        void saveEnvironmentalImpact(const string& filename, const RouteResults& results,
                                     WasteLocationManager* manager) {
            ofstream outFile(filename);
            
            if (!outFile) {
                cerr << "Cannot open file: " << filename << endl;
                return;
            }
            
            // Calculate environmental impact metrics
            double fuelSaved = calculateFuelSavings(results);
            double carbonReduction = fuelSaved * 2.3; // kg CO2 per liter of fuel
            double timeReduction = calculateTimeReduction(results);
            
            outFile << "Environmental Impact Report\n";
            outFile << "===================================\n\n";
            
            outFile << "External Factors Analysis:\n";
            outFile << "- Weather Condition Factor: " << fixed << setprecision(2) << weatherFactor 
                    << " (Weight: " << WEATHER_WEIGHT << ")\n";
            outFile << "- Traffic Congestion Factor: " << trafficFactor 
                    << " (Weight: " << TRAFFIC_WEIGHT << ")\n";
            outFile << "- Time of Day Factor: " << timeOfDayFactor 
                    << " (Weight: " << TIME_OF_DAY_WEIGHT << ")\n";
            outFile << "- Seasonal Factor: " << seasonalFactor 
                    << " (Weight: " << SEASONAL_WEIGHT << ")\n";
            outFile << "- Road Condition Factor: " << roadConditionFactor 
                    << " (Weight: " << ROAD_CONDITION_WEIGHT << ")\n\n";
            
            outFile << "Optimization Benefits:\n";
            outFile << "- Estimated Fuel Savings: " << fixed << setprecision(2) << fuelSaved << " liters\n";
            outFile << "- Estimated Carbon Reduction: " << carbonReduction << " kg CO2\n";
            outFile << "- Estimated Time Savings: " << timeReduction << " minutes\n\n";
            
            outFile << "Recommendations:\n";
            if (weatherFactor > 0.6) {
                outFile << "- Consider rescheduling non-urgent collections due to adverse weather\n";
            }
            if (trafficFactor > 0.7) {
                outFile << "- Consider adjusting collection time to avoid peak traffic periods\n";
            }
            if (timeOfDayFactor > 0.6) {
                outFile << "- Recommended time window adjustment to optimize daylight operations\n";
            }
            
            outFile.close();
        }
        
        // Calculate approximate fuel savings compared to non-optimized route
        double calculateFuelSavings(const RouteResults& results) const {
            // Calculate baseline fuel usage (non-optimized)
            vector<WasteLocation> locations;
            double baselineFuel = results.totalDistance * 0.15; // Liters per km (approximate)
            
            // Estimate savings based on optimization factors
            double optimizationEffect = 1.0 - ((weatherFactor * WEATHER_WEIGHT) +
                                             (trafficFactor * TRAFFIC_WEIGHT) +
                                             (timeOfDayFactor * TIME_OF_DAY_WEIGHT));
            
            // Calculate savings (10-25% depending on optimization)
            double savingsRate = 0.10 + (optimizationEffect * 0.15);
            return baselineFuel * savingsRate;
        }
        
        // Calculate approximate time savings
        double calculateTimeReduction(const RouteResults& results) const {
            // Baseline time (minutes)
            double baselineTime = results.totalTime;
            
            // Estimate time savings based on optimization factors, especially traffic
            double trafficEffect = 1.0 - (trafficFactor * 0.8);
            double timeEffect = 1.0 - (timeOfDayFactor * 0.5);
            
            // Calculate time savings rate (5-30% depending on factors)
            double savingsRate = 0.05 + (trafficEffect * 0.15) + (timeEffect * 0.10);
            
            return baselineTime * savingsRate;
        }
        
        // Load historical external factor data (or initialize if none exists)
        void loadHistoricalData() {
            ifstream inFile("external_factors_history.dat");
            
            if (inFile) {
                string locationName;
                double weather, traffic, timeOfDay, seasonal, road;
                
                while (inFile >> locationName >> weather >> traffic >> timeOfDay >> seasonal >> road) {
                    vector<double> factors = {weather, traffic, timeOfDay, seasonal, road};
                    historicalData[locationName] = factors;
                }
            } else {
                // Initialize with default values if no file exists
                historicalData.clear();
            }
        }
        
        // Save current external factors to historical data
        void updateHistoricalData(const vector<WasteLocation>& locations) {
            // Update the historical data with current values
            for (const auto& location : locations) {
                string name = location.getName();
                vector<double> factors = {weatherFactor, trafficFactor, timeOfDayFactor, 
                                        seasonalFactor, roadConditionFactor};
                historicalData[name] = factors;
            }
            
            // Save to file
            ofstream outFile("external_factors_history.dat");
            
            if (outFile) {
                for (const auto& entry : historicalData) {
                    outFile << entry.first;
                    for (double factor : entry.second) {
                        outFile << " " << factor;
                    }
                    outFile << endl;
                }
            }
        }
    
    public:
        ExternalFactorsRoute() {
            // Initialize factors with current conditions
            weatherFactor = getCurrentWeatherFactor();
            trafficFactor = getCurrentTrafficFactor();
            timeOfDayFactor = getTimeOfDayFactor();
            seasonalFactor = getSeasonalFactor();
            roadConditionFactor = getRoadConditionFactor();
            
            // Load historical data
            loadHistoricalData();
            
            // Initialize path cache
            pathCache.clear();
        }
        
        ~ExternalFactorsRoute() {
            // Save historical data when destroyed
            if (!historicalData.empty()) {
                ofstream outFile("external_factors_history.dat");
                if (outFile) {
                    for (const auto& entry : historicalData) {
                        outFile << entry.first;
                        for (double factor : entry.second) {
                            outFile << " " << factor;
                        }
                        outFile << endl;
                    }
                }
            }
        }
        
        RouteResults calculateRoute(WasteLocationManager* manager) override {
            RouteResults results;
            results.totalDistance = 0;
            results.totalTime = 0;
            results.totalFuel = 0;
            results.totalWage = 0;
            
            vector<WasteLocation>& locations = manager->getLocations();
            
            // Update the external factors based on current time and conditions
            weatherFactor = getCurrentWeatherFactor();
            trafficFactor = getCurrentTrafficFactor();
            timeOfDayFactor = getTimeOfDayFactor();
            seasonalFactor = getSeasonalFactor();
            roadConditionFactor = getRoadConditionFactor();
            
            // Initialize neural network if not already done
            if (neuralNetWeights.empty()) {
                initializeNeuralNetwork(locations.size());
            }
            
            // Create a list of locations to visit based on waste levels
            // Criteria: waste level >= 25% (like TSP, but with external factor optimization)
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
            
            // Apply the optimization algorithm to get the optimal path
            vector<int> optimizedPath = optimizeRoute(locationsToVisit, manager, locations);
            
            // Set the results path based on the optimized path
            results.path = optimizedPath;
            
            // Calculate costs using the original distance matrix (not adjusted distances)
            for (size_t i = 0; i < results.path.size() - 1; ++i) {
                int from = results.path[i];
                int to = results.path[i + 1];
                int legDistance = manager->getDistance(from, to);
                
                // Apply external factor adjustments to time calculation
                double combinedFactor = (weatherFactor * WEATHER_WEIGHT) +
                                     (trafficFactor * TRAFFIC_WEIGHT) +
                                     (timeOfDayFactor * TIME_OF_DAY_WEIGHT) +
                                     (seasonalFactor * SEASONAL_WEIGHT) +
                                     (roadConditionFactor * ROAD_CONDITION_WEIGHT);
                
                // Adjust time based on external factors (worse conditions = longer time)
                double timeAdjustment = 1.0 + (combinedFactor * 0.5);
                double legTime = legDistance * TIME_PER_KM * timeAdjustment;
                
                // Adjust fuel based on external factors (worse conditions = more fuel)
                double fuelAdjustment = 1.0 + (combinedFactor * 0.3);
                double legFuel = legDistance * FUEL_COST_PER_KM * fuelAdjustment;
                
                double legWage = (legTime / 60) * WAGE_PER_HOUR;
                
                results.totalDistance += legDistance;
                results.totalTime += legTime;
                results.totalFuel += legFuel;
                results.totalWage += legWage;
            }
            
            // Update historical data with current external factors
            updateHistoricalData(locations);
            
            // Generate environmental impact report
            saveEnvironmentalImpact("environmental_impact.txt", results, manager);
            
            return results;
        }
        
        // Get current external factors for display
        void getExternalFactors(double& weather, double& traffic, double& timeOfDay, 
                              double& seasonal, double& road) const {
            weather = weatherFactor;
            traffic = trafficFactor;
            timeOfDay = timeOfDayFactor;
            seasonal = seasonalFactor;
            road = roadConditionFactor;
        }
    };

/**
 * @class CollectionRoute
 * 
 * @brief A class that represents a collection route with a strategy for calculating the optimal route.
 * 
 * The CollectionRoute class is responsible for executing a collection route using a specified strategy.
 * It allows the user to set a strategy, execute the route, calculate the optimal route, print route information,
 * and save the route information to a file.
 * 
 * The class also provides a method to get the name of the current strategy being used.
 */// Collection Route class using Strategy pattern
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
        } else if (dynamic_cast<ExternalFactorsRoute*>(strategy.get())) {
            return "External Factors AI-based";
        } else {
            return "Unknown Strategy";
        }
    }

    shared_ptr<RouteStrategy> getStrategy() const {
        return strategy;
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

//The `WasteManagementSystem` class is a system for managing waste collection. It includes various functionalities such as generating random waste levels, viewing waste location information, selecting route algorithms, executing selected routes, saving location information to a file, viewing AI predictions, comparing route costs, displaying a waste pattern analytics dashboard, providing help documentation, saving all data, loading data, and deleting saved data.

//To use the system, create an instance of the `WasteManagementSystem` class and call the `run` method. This will display a menu for the user to select different options and perform the corresponding actions.

//The class includes private methods for displaying the system header, menu, and other UI elements. It also includes methods for selecting a route algorithm, getting the current route algorithm, viewing AI predictions, showing help documentation, confirming an action, saving data, loading data, and deleting data.

//The class also includes a map to store route results and a method to compare routes and display the cost savings analysis. Additionally, there are methods for displaying the waste pattern analytics dashboard and visualizing the route on a map.

//Please refer to the code for more details on the implementation of each method.// Main User Interface Class
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
        cout << "8. Select Most Cost-Effective Route" << endl;
        cout << "9. Waste Pattern Analytics Dashboard" << endl;
        cout << "10. View Collection History" << endl;
        cout << "11. Help" << endl;
        cout << "12. Save ALL Data" << endl; 
        cout << "13. Load ALL Data" << endl; 
        cout << "14. Delete ALL Data" << endl; 
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
        cout << "7. External Factors AI-based Route" << endl;
        cout << "8. Return to Main Menu" << endl;
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
                route.setStrategy(make_shared<ExternalFactorsRoute>());
                currentRoute = 7;
                break;
            case 8:
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
            case 7: return "External Factors AI-based";
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
        cout << "\n===============================================================" << endl;
        cout << "                   HELP DOCUMENTATION                          " << endl;
        cout << "===============================================================" << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n----- CORE FEATURES -----" << endl;
        
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
        cout << "   Choose from 7 different routing algorithms:" << endl;
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
        cout << "   - External Factors AI-based: Optimizes routes based on external factors" << endl;
        cout << "     like weather, traffic, time of day, and seasonal variations" << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n4. Execute Selected Route:" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "   Calculates and displays the optimized collection route using the" << endl;
        cout << "   selected algorithm. Shows distance, time, fuel cost, and wage cost." << endl;
        cout << "   The route information is also saved to 'route_info.txt'." << endl;
        cout << "   You can also visualize the route on a map after execution." << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n----- ANALYTICS & AI FEATURES -----" << endl;
        
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
    
        cout << "\n8. Select Most Cost-Effective Route:" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "   Automatically selects the most cost-effective route strategy" << endl;
        cout << "   based on previous route comparisons and simulates waste collection." << endl;
        cout << "   This feature requires executing at least 2 different routes to compare costs." << endl;
        cout << "   After selecting the most efficient route, you can simulate waste collection," << endl;
        cout << "   which will set the waste levels of visited locations to 0." << endl;
        setTextColor(COLOR_WHITE);
    
        cout << "\n9. Waste Pattern Analytics Dashboard:" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "   Comprehensive dashboard that analyzes historical waste data," << endl;
        cout << "   identifies patterns, and provides actionable insights including:" << endl;
        cout << "   - Days until bins reach capacity" << endl;
        cout << "   - Waste pattern classification" << endl;
        cout << "   - Trend visualization" << endl;
        cout << "   - Prioritized recommendations" << endl;
        cout << "   - Visual map representation of waste locations and routes" << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n10. View Collection History:" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "   Displays a detailed history of all waste collections performed," << endl;
        cout << "   including dates, times, and amounts collected at each location." << endl;
        cout << "   Options to export the collection history to a text file." << endl;
        cout << "   This data is valuable for long-term waste trend analysis." << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n----- DATA MANAGEMENT -----" << endl;
        
        cout << "\n12. Save ALL Data:" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "   Saves all current waste data, including locations, waste levels," << endl;
        cout << "   historical data, collection history, and distance matrix to a binary file." << endl;
        cout << "   Creates a bundle file containing all supplementary data files." << endl;
        cout << "   A manifest file is also created to track all saved components." << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n13. Load ALL Data:" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "   Loads previously saved waste data from binary files." << endl;
        cout << "   Options to load only core data or all data including supplementary files." << endl;
        cout << "   This restores all locations, waste levels, historical data, and collection history." << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n14. Delete ALL Data:" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "   Permanently deletes saved data files." << endl;
        cout << "   Options to delete only core data or all related data files." << endl;
        cout << "   Use with caution as this operation cannot be undone." << endl;
        setTextColor(COLOR_WHITE);
        
        cout << "\n----- SYSTEM OPERATIONS -----" << endl;
        
        cout << "\n11. Help: Display this help information" << endl;
        
        cout << "\n0. Exit: Close the application" << endl;
        
        cout << "\n----- ADDITIONAL INFORMATION -----" << endl;
        setTextColor(COLOR_YELLOW);
        cout << "Cost calculations include both fuel (RM 2.50/km) and driver wages (RM 10.00/hour)" << endl;
        cout << "Route comparisons help identify the most cost-effective collection strategy" << endl;
        cout << "Anomalies are detected when waste levels deviate significantly from historical trends" << endl;
        cout << "All routes begin and end at the Waste Collector HQ" << endl;
        cout << "The External Factors route considers weather, traffic, time of day and seasonal variations" << endl;
        cout << "The RL (Reinforcement Learning) route improves over time as more routes are executed" << endl;
        cout << "Your data is saved to 'waste_data.dat' by default" << endl;
        
        setTextColor(COLOR_GREEN);
        cout << "\n----- TIPS FOR OPTIMAL USE -----" << endl;
        cout << "Execute multiple route types before comparing costs" << endl;
        cout << "Check the analytics dashboard regularly to identify problematic locations" << endl;
        cout << "Consider using the 'RL Route' for long-term optimization" << endl;
        cout << "Always save your data before closing the application" << endl;
        cout << "The External Factors route provides significant benefits during bad weather or traffic" << endl;
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
        
        cout << "\nSave ALL Data Options:" << endl;  // Updated title
        cout << "1. Save to default file (" << locationManager->getLastSavedFilePath() << ")" << endl;
        cout << "2. Save to a custom file" << endl;
        cout << "3. Return to main menu" << endl;
        cout << "\nEnter your choice: ";
    
    int choice;
    cin >> choice;
    
    string targetFilename;
    
    switch (choice) {
        case 1: {
            // Save to default file
            targetFilename = locationManager->getLastSavedFilePath();
            break;
        }
        case 2: {
            // Save to custom file
            cout << "Enter the filename to save to (e.g., mydata.dat): ";
            cin >> targetFilename;
            break;
        }
        case 3:
            // Return to main menu
            return;
        default:
            cout << "Invalid choice. Please try again." << endl;
            system("pause");
            return;
    }
    
    try {
        UIHelper::displayProgressBar(0);
        cout << "\nSaving ALL data files to " << targetFilename << "..." << endl;

        // Save core waste location data first using the existing method
        bool coreDataSaved = locationManager->saveDataToFile(true, targetFilename);
        if (!coreDataSaved) {
            throw WasteManagementException("Failed to save core waste location data");
        }
        UIHelper::displayProgressBar(30);

        // Create a manifest file to track all additional files
        string manifestFilename = targetFilename + "_manifest.txt";
        ofstream manifestFile(manifestFilename);
        if (!manifestFile) {
            throw FileOperationException("create", manifestFilename);
        }
        
        // List of files to check and save
        vector<string> filesToSave = {
            "environmental_impact.txt",
            "external_factors_history.dat",
            "locations_info.txt",
            "qtable.txt",
            "route_info.txt",
            "collection_history.txt",
            "route_comparison.txt",         // Added route comparison file
            "most_cost_effective_route.txt" // Added most cost-effective route file
        };
        
        manifestFile << "Waste Management System - File Manifest\n";
        manifestFile << "Created: " << __DATE__ << " " << __TIME__ << "\n";
        manifestFile << "========================================\n\n";
        manifestFile << "Core data file: " << targetFilename << "\n\n";
        manifestFile << "Additional files:\n";
        
        int filesFound = 0;
        int progress = 30;
        int progressStep = 60 / filesToSave.size();
        
        // Create a comprehensive bundle file
        string bundleFilename = targetFilename + "_bundle.txt";
        ofstream bundleFile(bundleFilename, ios::binary);
        if (!bundleFile) {
            throw FileOperationException("create", bundleFilename);
        }
        
        bundleFile << "WASTE_MANAGEMENT_SYSTEM_BUNDLE_START\n";
        bundleFile << "VERSION: 1.0\n";
        bundleFile << "CREATION_DATE: " << __DATE__ << " " << __TIME__ << "\n";
        bundleFile << "FILE_COUNT: " << filesToSave.size() + 1 << "\n"; // +1 for core data
        bundleFile << "========================================\n\n";
        
        // Save core file reference first
        bundleFile << "FILE_START: " << targetFilename << "\n";
        bundleFile << "FILE_TYPE: CORE_DATA\n";
        bundleFile << "FILE_END\n\n";
        
        // Check each file and add to the bundle
        for (const string& filename : filesToSave) {
            ifstream fileCheck(filename);
            progress += progressStep;
            UIHelper::displayProgressBar(progress);
            
            if (fileCheck) {
                // File exists, add to manifest
                filesFound++;
                manifestFile << "- " << filename << " (Included)\n";
                
                // Add file content to bundle
                bundleFile << "FILE_START: " << filename << "\n";
                bundleFile << "FILE_TYPE: " << (filename.substr(filename.find_last_of(".") + 1) == "dat" ? "BINARY" : "TEXT") << "\n";
                bundleFile << "CONTENT_START\n";
                
                // Read and write the entire file content
                fileCheck.seekg(0, ios::end);
                size_t fileSize = fileCheck.tellg();
                fileCheck.seekg(0, ios::beg);
                
                char* buffer = new char[fileSize];
                fileCheck.read(buffer, fileSize);
                bundleFile.write(buffer, fileSize);
                delete[] buffer;
                
                bundleFile << "\nCONTENT_END\n";
                bundleFile << "FILE_END\n\n";
            } else {
                manifestFile << "- " << filename << " (Not found)\n";
            }
        }
        
        bundleFile << "WASTE_MANAGEMENT_SYSTEM_BUNDLE_END\n";
        bundleFile.close();
        
        manifestFile << "\nTotal files found and saved: " << filesFound + 1 << " (including core data)\n"; 
        manifestFile << "Bundle file: " << bundleFilename << "\n";
        manifestFile.close();
        
        UIHelper::displayProgressBar(100);
        cout << endl;
        UIHelper::displaySuccess("All data saved successfully!");
        cout << "- Core data saved to: " << targetFilename << endl;
        cout << "- Additional files bundled in: " << bundleFilename << endl;
        cout << "- Manifest created at: " << manifestFilename << endl;
        cout << "Total files saved: " << filesFound + 1 << " (including core data)" << endl;
    }
    catch (const exception& e) {
        UIHelper::displayError(e.what());
    }
    
    system("pause");
}

//loadData function to handle the bundle
void loadData() {
    system("cls");
    displayHeader();
    
    cout << "\nLoad ALL Data Options:" << endl;  // Updated title
    cout << "1. Load core data only (from " << locationManager->getLastSavedFilePath() << ")" << endl;
    cout << "2. Load ALL data (core + additional files from bundle)" << endl;  // Clearer option
    cout << "3. Load from a custom file" << endl;
    cout << "4. Return to main menu" << endl;
    cout << "\nEnter your choice: ";
    
    int choice;
    cin >> choice;
    
    string targetFilename;
    bool loadBundle = false;
    
    switch (choice) {
        case 1: {
            // Load core data only
            targetFilename = locationManager->getLastSavedFilePath();
            loadBundle = false;
            break;
        }
        case 2: {
            // Load all data including bundle
            targetFilename = locationManager->getLastSavedFilePath();
            loadBundle = true;
            break;
        }
        case 3: {
            // Load from custom file
            cout << "Enter the filename to load from: ";
            cin >> targetFilename;
            
            cout << "Would you like to load additional files from bundle? (y/n): ";
            char bundleChoice;
            cin >> bundleChoice;
            loadBundle = (bundleChoice == 'y' || bundleChoice == 'Y');
            break;
        }
        case 4:
            // Return to main menu
            return;
        default:
            cout << "Invalid choice. Please try again." << endl;
            system("pause");
            return;
    }
    
    try {
        if (!locationManager->dataFileExists(targetFilename)) {
            throw FileOperationException("find", targetFilename);
        }
        
        if (confirmAction("This will overwrite current data. Continue?")) {
            // First load the core data
            UIHelper::displayProgressBar(0);
            cout << "\nLoading core data from " << targetFilename << "..." << endl;
            
            bool coreDataLoaded = locationManager->loadAllData(targetFilename);
            if (!coreDataLoaded) {
                throw WasteManagementException("Failed to load core waste location data");
            }
            UIHelper::displayProgressBar(50);
            
            // If bundle loading is requested, load the additional files
            if (loadBundle) {
                string bundleFilename = targetFilename + "_bundle.txt";
                ifstream bundleFile(bundleFilename, ios::binary);
                
                if (!bundleFile) {
                    UIHelper::displayWarning("Bundle file not found: " + bundleFilename);
                    UIHelper::displayWarning("Only core data was loaded");
                    UIHelper::displayProgressBar(100);
                    cout << endl;
                    UIHelper::displaySuccess("Core data loaded successfully from " + targetFilename);
                    system("pause");
                    return;
                }
                
                cout << "Loading additional files from bundle..." << endl;
                
                string line;
                string currentFile;
                bool inFile = false;
                bool inContent = false;
                ofstream outputFile;
                
                // Skip header until we find the first FILE_START
                while (getline(bundleFile, line)) {
                    if (line.find("FILE_START:") != string::npos) {
                        // Extract filename
                        currentFile = line.substr(line.find(":") + 1);
                        // Trim whitespace
                        currentFile.erase(0, currentFile.find_first_not_of(" \t"));
                        currentFile.erase(currentFile.find_last_not_of(" \t") + 1);
                        
                        // Skip the core data file which we've already loaded
                        if (currentFile == targetFilename) {
                            inFile = true;
                            inContent = false;
                            continue;
                        }
                        
                        cout << "Extracting: " << currentFile << endl;
                        outputFile.open(currentFile, ios::binary);
                        if (!outputFile) {
                            UIHelper::displayWarning("Could not create file: " + currentFile);
                            inFile = true;
                            inContent = false;
                            continue;
                        }
                        
                        inFile = true;
                        inContent = false;
                    }
                    else if (inFile && line.find("CONTENT_START") != string::npos) {
                        inContent = true;
                    }
                    else if (inFile && line.find("CONTENT_END") != string::npos) {
                        inContent = false;
                    }
                    else if (inFile && line.find("FILE_END") != string::npos) {
                        inFile = false;
                        inContent = false;
                        if (outputFile.is_open()) {
                            outputFile.close();
                        }
                    }
                    else if (inFile && inContent) {
                        // Write content to the file
                        if (outputFile.is_open()) {
                            outputFile << line << endl;
                        }
                    }
                }
                
                if (outputFile.is_open()) {
                    outputFile.close();
                }
                
                // Check if route comparison and most cost-effective route data were loaded
                bool routeComparisonLoaded = loadRouteComparisonData();
                bool costEffectiveRouteLoaded = loadMostCostEffectiveRouteData();

                UIHelper::displayProgressBar(100);
                cout << endl;
                UIHelper::displaySuccess("All data loaded successfully!");
                cout << "- Core data loaded from: " << targetFilename << endl;
                cout << "- Additional files extracted from bundle" << endl;
            } else {
                UIHelper::displayProgressBar(100);
                cout << endl;
                UIHelper::displaySuccess("Core data loaded successfully from " + targetFilename);
            }
        }
    }
    catch (const exception& e) {
        UIHelper::displayError(e.what());
    }
    
    system("pause");
}

// Update the deleteData function to handle bundles
void deleteData() {
    system("cls");
    displayHeader();
    
    cout << "\nDelete ALL Data Options:" << endl;  // Updated title
    cout << "1. Delete core data file only (" << locationManager->getLastSavedFilePath() << ")" << endl;
    cout << "2. Delete ALL data files (core + bundle + manifest)" << endl;  // Clearer option
    cout << "3. Delete a custom file" << endl;
    cout << "4. Return to main menu" << endl;
    cout << "\nEnter your choice: ";
    
    int choice;
    cin >> choice;
    
    string targetFilename;
    bool deleteAll = false;
    
    switch (choice) {
        case 1: {
            // Delete core file only
            targetFilename = locationManager->getLastSavedFilePath();
            deleteAll = false;
            break;
        }
        case 2: {
            // Delete all related files
            targetFilename = locationManager->getLastSavedFilePath();
            deleteAll = true;
            break;
        }
        case 3: {
            // Delete custom file
            cout << "Enter the filename to delete: ";
            cin >> targetFilename;
            
            cout << "Delete related bundle and manifest files too? (y/n): ";
            char allChoice;
            cin >> allChoice;
            deleteAll = (allChoice == 'y' || allChoice == 'Y');
            break;
        }
        case 4:
            // Return to main menu
            return;
        default:
            cout << "Invalid choice. Please try again." << endl;
            system("pause");
            return;
    }
    
    try {
        if (!locationManager->dataFileExists(targetFilename)) {
            throw FileOperationException("find", targetFilename);
        }
        
        if (confirmAction("Are you sure you want to delete these files? This cannot be undone.")) {
            // Delete the core data file
            bool coreDeleted = locationManager->deleteDataFile(targetFilename);
            if (!coreDeleted) {
                throw WasteManagementException("Failed to delete core data file");
            }
            
            cout << "Core data file deleted: " << targetFilename << endl;
            
            // If requested, delete bundle and manifest files too
            if (deleteAll) {
                string bundleFilename = targetFilename + "_bundle.txt";
                string manifestFilename = targetFilename + "_manifest.txt";
                
                // Try to delete bundle file
                ifstream bundleCheck(bundleFilename);
                if (bundleCheck) {
                    bundleCheck.close();
                    if (remove(bundleFilename.c_str()) != 0) {
                        UIHelper::displayWarning("Could not delete bundle file: " + bundleFilename);
                    } else {
                        cout << "Bundle file deleted: " << bundleFilename << endl;
                    }
                }
                
                // Try to delete manifest file
                ifstream manifestCheck(manifestFilename);
                if (manifestCheck) {
                    manifestCheck.close();
                    if (remove(manifestFilename.c_str()) != 0) {
                        UIHelper::displayWarning("Could not delete manifest file: " + manifestFilename);
                    } else {
                        cout << "Manifest file deleted: " << manifestFilename << endl;
                    }
                }
                
                // Delete additional files that were in the bundle
                vector<string> additionalFiles = {
                    "environmental_impact.txt",
                    "external_factors_history.dat",
                    "locations_info.txt",
                    "qtable.txt",
                    "route_info.txt",
                    "collection_history.txt",
                    "route_comparison.txt",          
                    "most_cost_effective_route.txt"  
                };
                
                cout << "\nDeleting additional files..." << endl;
                for (const auto& file : additionalFiles) {
                    ifstream fileCheck(file);
                    if (fileCheck) {
                        fileCheck.close();
                        if (remove(file.c_str()) != 0) {
                            UIHelper::displayWarning("Could not delete file: " + file);
                        } else {
                            cout << "File deleted: " << file << endl;
                        }
                    }
                }
            }
            
            UIHelper::displaySuccess("Data files deleted successfully");
        }
    }
    catch (const exception& e) {
        UIHelper::displayError(e.what());
    }
    
    system("pause");
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

        // Save route comparison data to file automatically
        saveRouteComparisonData();

        // Add option to select the most cost-effective route and collect waste
    cout << "\nOptions:" << endl;
    cout << "1. Select the most cost-effective route" << endl;
    cout << "2. Return to main menu" << endl;
    cout << "Enter your choice: ";
    
    int choice;
    cin >> choice;
    
    if (choice == 1) {
        selectMostCostEffectiveRoute();
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

private:
void selectMostCostEffectiveRoute() {
    if (routeResults.size() < 2) {
        UIHelper::displayWarning("You need to execute at least 2 different routes to find the most cost-effective one.");
        return;
    }
    
    // Find the cheapest route
    string cheapestRoute = "";
    double cheapestCost = DBL_MAX;
    RouteResults bestRouteResults;
    int cheapestRouteIndex = 0;
    
    cout << "\nFinding the most cost-effective route..." << endl;
    
    for (const auto& result : routeResults) {
        double totalCost = result.second.totalFuel + result.second.totalWage;
        if (totalCost < cheapestCost) {
            cheapestCost = totalCost;
            cheapestRoute = result.first;
            bestRouteResults = result.second;
            
            // Determine the route index for setting the strategy
            if (cheapestRoute == "Regular (Non-Optimized)") cheapestRouteIndex = 1;
            else if (cheapestRoute == "Optimized") cheapestRouteIndex = 2;
            else if (cheapestRoute == "Greedy") cheapestRouteIndex = 3;
            else if (cheapestRoute == "Traveling Salesman Problem") cheapestRouteIndex = 4;
            else if (cheapestRoute == "Minimum Spanning Tree") cheapestRouteIndex = 5;
            else if (cheapestRoute == "Reinforcement Learning") cheapestRouteIndex = 6;
            else if (cheapestRoute == "External Factors AI-based") cheapestRouteIndex = 7;
        }
    }
    
    // Switch to the most cost-effective route
    switch (cheapestRouteIndex) {
        case 1:
            route.setStrategy(make_shared<NonOptimizedRoute>());
            break;
        case 2:
            route.setStrategy(make_shared<OptimizedRoute>());
            break;
        case 3:
            route.setStrategy(make_shared<GreedyRoute>());
            break;
        case 4:
            route.setStrategy(make_shared<TSPRoute>());
            break;
        case 5:
            route.setStrategy(make_shared<MSTRoute>());
            break;
        case 6:
            route.setStrategy(make_shared<RLRoute>());
            break;
        case 7:
            route.setStrategy(make_shared<ExternalFactorsRoute>());
            break;
        default:
            UIHelper::displayError("Could not determine the most cost-effective route.");
            return;
    }
    
    // Update the current route
    currentRoute = cheapestRouteIndex;
    
    UIHelper::displaySuccess("Selected the most cost-effective route: " + cheapestRoute);
    
    cout << "\nTotal cost: " << fixed << setprecision(2) << cheapestCost 
         << " RM (Fuel: " << bestRouteResults.totalFuel 
         << " RM, Wages: " << bestRouteResults.totalWage << " RM)" << endl;

    // Save most cost-effective route data to file
    saveMostCostEffectiveRouteData(cheapestRoute, bestRouteResults);
    
    // Ask if user wants to simulate waste collection using this route
    cout << "\nDo you want to simulate waste collection using this route? (y/n): ";
    char choice;
    cin >> choice;
    
    if (choice == 'y' || choice == 'Y') {
        simulateWasteCollection(bestRouteResults);
    }
}

// Method to simulate waste collection by setting waste levels to 0 for visited locations
void simulateWasteCollection(const RouteResults& results) {
    if (results.path.empty()) {
        UIHelper::displayWarning("No route to follow for waste collection.");
        return;
    }
    
    vector<WasteLocation>& locations = locationManager->getLocations();
    
    cout << "\nSimulating waste collection along the selected route..." << endl;
    cout << "===========================================================" << endl;
    cout << "Location                    | Before | After " << endl;
    cout << "-----------------------------------------------------------" << endl;
    
    // Track how many locations were actually collected (excluding HQ and duplicate visits)
    int locationsCollected = 0;
    unordered_set<int> collectedLocations;
    
    // Skip the first element (HQ) and process all other locations in the path
    for (size_t i = 1; i < results.path.size(); i++) {
        int locationIndex = results.path[i];
        
        // Skip HQ (index 0) and already collected locations
        if (locationIndex == 0 || collectedLocations.find(locationIndex) != collectedLocations.end()) {
            continue;
        }
        
        // Store the previous waste level for reporting
        int previousWasteLevel = locations[locationIndex].getWasteLevel();
        
        // Only perform collection if the waste level is significant
        if (previousWasteLevel >= 25) {  // Using 25% as the minimum threshold
            // Record the collection with the amount collected (previous waste level)
            locations[locationIndex].recordCollection(previousWasteLevel);
            
            // Set waste level to 0 (collected)
            locations[locationIndex].setWasteLevel(0);
            locations[locationIndex].setIsCollected(true);
            
            // Print the before/after waste levels
            cout << left << setw(28) << locations[locationIndex].getName() 
                 << "| " << setw(7) << previousWasteLevel << "% | 0%" << endl;
            
            locationsCollected++;
            collectedLocations.insert(locationIndex);
        }
    }
    
    cout << "===========================================================" << endl;
    cout << "Total locations visited: " << results.path.size() - 1 << endl;  // Exclude the return to HQ
    cout << "Total locations collected: " << locationsCollected << endl;
    
    UIHelper::displaySuccess("Waste collection simulation completed successfully!");
}

    // Save route comparison data to file
    void saveRouteComparisonData() {
        ofstream outFile("route_comparison.txt");
        
        if (!outFile) {
            UIHelper::displayError("Cannot open file: route_comparison.txt");
            return;
        }
        
        // Write header
        outFile << "WASTE MANAGEMENT SYSTEM - ROUTE COMPARISON DATA" << endl;
        outFile << "===============================================" << endl;
        time_t now = time(nullptr);
        outFile << "Generated: " << ctime(&now);
        outFile << endl;
        
        // Write route comparison table
        outFile << left << setw(30) << "Route Type" 
             << setw(15) << "Distance (km)" 
             << setw(15) << "Time (h)" 
             << setw(15) << "Fuel (RM)" 
             << setw(15) << "Wages (RM)" 
             << setw(15) << "Total (RM)" << endl;
        outFile << string(105, '-') << endl;
        
        for (const auto& route : routeResults) {
            double totalCost = route.second.totalFuel + route.second.totalWage;
            outFile << left << setw(30) << route.first
                 << setw(15) << route.second.totalDistance
                 << setw(15) << fixed << setprecision(2) << (route.second.totalTime / 60)
                 << setw(15) << fixed << setprecision(2) << route.second.totalFuel
                 << setw(15) << fixed << setprecision(2) << route.second.totalWage
                 << setw(15) << fixed << setprecision(2) << totalCost << endl;
        }
        
        // Find the cheapest route
        string cheapestRoute = "";
        double cheapestCost = DBL_MAX;
        
        for (const auto& route : routeResults) {
            double totalCost = route.second.totalFuel + route.second.totalWage;
            if (totalCost < cheapestCost) {
                cheapestCost = totalCost;
                cheapestRoute = route.first;
            }
        }
        
        // Write the cost savings analysis
        outFile << endl << "COST SAVINGS ANALYSIS" << endl;
        outFile << "=====================" << endl;
        outFile << "Most cost-effective route: " << cheapestRoute << endl;
        
        for (const auto& route : routeResults) {
            if (route.first != cheapestRoute) {
                double routeCost = route.second.totalFuel + route.second.totalWage;
                double savings = routeCost - cheapestCost;
                double savingsPercent = (savings / routeCost) * 100;
                
                outFile << "Switching from " << route.first << " to " << cheapestRoute 
                     << " saves " << fixed << setprecision(2) << savings 
                     << " RM (" << savingsPercent << "% reduction)" << endl;
            }
        }
        
        outFile.close();
        UIHelper::displaySuccess("Route comparison data saved to 'route_comparison.txt'");
    }
    
    // Save most cost-effective route data
    void saveMostCostEffectiveRouteData(const string& routeName, const RouteResults& results) {
        ofstream outFile("most_cost_effective_route.txt");
        
        if (!outFile) {
            UIHelper::displayError("Cannot open file: most_cost_effective_route.txt");
            return;
        }
        
        // Write header
        outFile << "WASTE MANAGEMENT SYSTEM - MOST COST-EFFECTIVE ROUTE" << endl;
        outFile << "==================================================" << endl;
        time_t now = time(nullptr);
        outFile << "Generated: " << ctime(&now);
        outFile << endl;
        
        // Write most cost-effective route info
        outFile << "Selected Route: " << routeName << endl;
        outFile << "Total Distance: " << results.totalDistance << " km" << endl;
        outFile << "Total Time: " << fixed << setprecision(2) << (results.totalTime / 60) << " hours" << endl;
        outFile << "Total Fuel Cost: " << fixed << setprecision(2) << results.totalFuel << " RM" << endl;
        outFile << "Total Wages: " << fixed << setprecision(2) << results.totalWage << " RM" << endl;
        outFile << "Total Cost: " << fixed << setprecision(2) << (results.totalFuel + results.totalWage) << " RM" << endl;
        
        // Write route path
        outFile << endl << "ROUTE PATH:" << endl;
        outFile << "===========" << endl;
        
        vector<WasteLocation>& locations = locationManager->getLocations();
        for (size_t i = 0; i < results.path.size(); i++) {
            outFile << locations[results.path[i]].getName();
            if (i < results.path.size() - 1) {
                outFile << " -> ";
            }
        }
        outFile << endl;
        
        outFile.close();
        UIHelper::displaySuccess("Most cost-effective route data saved to 'most_cost_effective_route.txt'");
    }

bool loadRouteComparisonData() {    
    ifstream inFile("route_comparison.txt");
    
    if (!inFile) {
        UIHelper::displayWarning("Route comparison data file not found or cannot be opened");
        return false;
    }
    
    // Clear existing route results
    routeResults.clear();
    
    // Skip header lines until we find the table
    string line;
    while (getline(inFile, line)) {
        if (line.find("Route Type") != string::npos && 
            line.find("Distance") != string::npos &&
            line.find("Time") != string::npos) {
            break;
        }
    }
    
    // Skip the separator line
    getline(inFile, line);
    
    // Read the route data
    while (getline(inFile, line) && !line.empty()) {
        istringstream iss(line);
        string routeName;
        int distance;
        double time, fuel, wage;
        
        // Parse the line - assuming fields are separated by whitespace
        iss >> routeName;
        
        // If route name has spaces, read until we find the distance
        string temp;
        while (iss >> temp) {
            if (isdigit(temp[0])) {
                distance = stoi(temp);
                break;
            }
            routeName += " " + temp;
        }
        
        iss >> time >> fuel >> wage;
        
        // Create a new RouteResults object
        RouteResults result;
        result.totalDistance = distance;
        result.totalTime = time * 60; // Convert hours back to minutes
        result.totalFuel = fuel;
        result.totalWage = wage;
        
        // Add to map
        routeResults[routeName] = result;
    }
    
    UIHelper::displaySuccess("Route comparison data loaded successfully!");
    inFile.close();
    return true;
}
    
    // Load most cost-effective route data
    bool loadMostCostEffectiveRouteData() {
        ifstream inFile("most_cost_effective_route.txt");
        
        if (!inFile) {
            UIHelper::displayWarning("Most cost-effective route data file not found or cannot be opened");
            return false;
        }
        
        UIHelper::displaySuccess("Most cost-effective route data found. This data can be accessed through the 'Select Most Cost-Effective Route' menu option.");
        inFile.close();
        return true;
    }

void displayCollectionHistory() {
    system("cls");
    displayHeader();
    
    const vector<WasteLocation>& locations = locationManager->getLocations();
    
    setTextColor(COLOR_BLUE);
    cout << "\n=================================================================" << endl;
    cout << "                     COLLECTION HISTORY                           " << endl;
    cout << "=================================================================" << endl;
    setTextColor(COLOR_WHITE);
    
    bool anyCollections = false;
    
    // First check if there are any collections
    for (size_t i = 1; i < locations.size(); i++) {
        if (!locations[i].getCollectionHistory().empty()) {
            anyCollections = true;
            break;
        }
    }
    
    if (!anyCollections) {
        UIHelper::displayWarning("No collection history available. Try simulating waste collection first.");
        cout << "\nPress any key to return to the main menu...";
        _getch();
        return;
    }
    
    // Display collection history for each location
    for (size_t i = 1; i < locations.size(); i++) {
        const vector<pair<time_t, double>>& history = locations[i].getCollectionHistory();
        
        if (history.empty()) continue;
        
        setTextColor(COLOR_GREEN);
        cout << "\n" << locations[i].getName() << ":" << endl;
        cout << string(locations[i].getName().length() + 1, '-') << endl;
        setTextColor(COLOR_WHITE);
        
        double totalCollected = 0;
        cout << left
             << setw(25) << "Date & Time"
             << setw(20) << "Amount Collected"
             << endl;
        cout << string(45, '-') << endl;
        
        // Sort history in reverse chronological order (newest first)
        vector<pair<time_t, double>> sortedHistory = history;
        sort(sortedHistory.begin(), sortedHistory.end(), 
             [](const pair<time_t, double>& a, const pair<time_t, double>& b) {
                 return a.first > b.first;
             });
        
        for (const auto& entry : sortedHistory) {
            // Format the date and time
            char buffer[30];
            struct tm* timeinfo = localtime(&entry.first);
            strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
            
            cout << left 
                 << setw(25) << buffer
                 << setw(20) << fixed << setprecision(2) << entry.second << "%" 
                 << endl;
                 
            totalCollected += entry.second;
        }
        
        cout << string(45, '-') << endl;
        cout << "Total collections: " << history.size() << endl;
        cout << "Total waste collected: " << fixed << setprecision(2) << totalCollected << "%" << endl;
    }
    
    // Option to export collection history
    cout << "\nOptions:" << endl;
    cout << "1. Export Collection History to File" << endl;
    cout << "2. Return to Main Menu" << endl;
    cout << "Enter your choice: ";
    
    int choice;
    cin >> choice;
    
    if (choice == 1) {
        ofstream outFile("collection_history.txt");
        if (!outFile) {
            UIHelper::displayError("Cannot open file: collection_history.txt");
        } else {
            outFile << "WASTE COLLECTION HISTORY REPORT" << endl;
            outFile << "============================" << endl;
            time_t currentTime = time(nullptr);
            outFile << "Generated on: " << ctime(&currentTime);
            outFile << endl;
            
            for (size_t i = 1; i < locations.size(); i++) {
                const vector<pair<time_t, double>>& history = locations[i].getCollectionHistory();
                
                if (history.empty()) continue;
                
                outFile << locations[i].getName() << ":" << endl;
                outFile << string(locations[i].getName().length() + 1, '-') << endl;
                
                outFile << left
                     << setw(25) << "Date & Time"
                     << setw(20) << "Amount Collected"
                     << endl;
                outFile << string(45, '-') << endl;
                
                double totalCollected = 0;
                
                // Sort history in reverse chronological order
                vector<pair<time_t, double>> sortedHistory = history;
                sort(sortedHistory.begin(), sortedHistory.end(), 
                     [](const pair<time_t, double>& a, const pair<time_t, double>& b) {
                         return a.first > b.first;
                     });
                
                for (const auto& entry : sortedHistory) {
                    char buffer[30];
                    struct tm* timeinfo = localtime(&entry.first);
                    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
                    
                    outFile << left 
                         << setw(25) << buffer
                         << setw(20) << fixed << setprecision(2) << entry.second << "%" 
                         << endl;
                         
                    totalCollected += entry.second;
                }
                
                outFile << string(45, '-') << endl;
                outFile << "Total collections: " << history.size() << endl;
                outFile << "Total waste collected: " << fixed << setprecision(2) << totalCollected << "%" << endl;
                outFile << endl;
            }
            
            outFile.close();
            UIHelper::displaySuccess("Collection history exported to 'collection_history.txt'");
        }
    }
    
    cout << "\nPress any key to return to the main menu...";
    _getch();
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
            
            if (cin.fail()) {
                cin.clear();  // Clear the error flag
                cin.ignore(10000, '\n');   // Discard invalid input
                cout << "Invalid input. Please enter a number." << endl;
                system("pause");
                continue;
            }
            
            switch (choice) {
                case 1:
                    // Generate random waste levels
                    locationManager->generateRandomWasteLevels();
                    cout << "\nRandom waste levels generated." << endl;
                    system("pause");
                    break;
                    
                case 2:
                    // View waste locations info
                    system("cls");
                    displayHeader();
                    locationManager->printLocationsInfo();
                    system("pause");
                    break;
                    
                case 3:
                    // Select route algorithm
                    selectRouteAlgorithm();
                    break;
                    
                case 4:
                    // Execute selected route
                    system("cls");
                    displayHeader();
                    try {
                        RouteResults results = route.executeRoute(locationManager);
                        
                        // Save the results for comparison
                        routeResults[getCurrentRouteAlgorithm()] = results;
                        
                        // Show route visualization
                        cout << "\nWould you like to see a visual map of the route? (y/n): ";
                        char mapChoice;
                        cin >> mapChoice;
                        
                        if (mapChoice == 'y' || mapChoice == 'Y') {
                            visualizeRouteMap(results, locationManager);
                        }
                        
                        // If using External Factors AI-based route, display external factors
                        if (currentRoute == 7) {
                            auto extRoute = dynamic_cast<ExternalFactorsRoute*>(route.getStrategy().get());
                            if (extRoute) {
                                double weather, traffic, timeOfDay, seasonal, road;
                                extRoute->getExternalFactors(weather, traffic, timeOfDay, seasonal, road);
                                
                                cout << "\nExternal Factors Analysis:" << endl;
                                cout << "------------------------" << endl;
                                cout << "Weather Condition: " << fixed << setprecision(2) << weather * 100 << "% impact" << endl;
                                cout << "Traffic Congestion: " << traffic * 100 << "% impact" << endl;
                                cout << "Time of Day Factor: " << timeOfDay * 100 << "% impact" << endl;
                                cout << "Seasonal Factor: " << seasonal * 100 << "% impact" << endl;
                                cout << "Road Condition: " << road * 100 << "% impact" << endl;
                                
                                cout << "\nEnvironmental impact analysis saved to 'environmental_impact.txt'" << endl;
                            }
                        }
                    }
                    catch (const InvalidRouteException& e) {
                        UIHelper::displayError(e.what());
                    }
                    system("pause");
                    break;
                    
                case 5:
                    // Save locations info to file
                    {
                        ofstream outFile("locations_info.txt");
                        if (!outFile) {
                            UIHelper::displayError("Cannot open file: locations_info.txt");
                            system("pause");
                            break;
                        }
                        
                        const vector<WasteLocation>& locations = locationManager->getLocations();
                        
                        outFile << "Waste Locations Information" << endl;
                        outFile << "===========================" << endl << endl;
                        
                        outFile << left 
                            << setw(30) << "Location" 
                            << setw(20) << "Waste Level %" 
                            << endl;
                        outFile << string(50, '-') << endl;
                        
                        for (const auto& loc : locations) {
                            outFile << left 
                                << setw(30) << loc.getName() 
                                << setw(20) << loc.getWasteLevel() 
                                << endl;
                        }
                        
                        outFile << endl << "Distance Matrix (km)" << endl;
                        outFile << "====================" << endl << endl;
                        
                        // Header row
                        outFile << setw(20) << "From\\To";
                        for (const auto& loc : locations) {
                            outFile << setw(15) << loc.getName();
                        }
                        outFile << endl;
                        
                        // Distance matrix
                        for (size_t i = 0; i < locations.size(); i++) {
                            outFile << setw(20) << locations[i].getName();
                            for (size_t j = 0; j < locations.size(); j++) {
                                outFile << setw(15) << locationManager->getDistance(i, j);
                            }
                            outFile << endl;
                        }
                        
                        outFile.close();
                        
                        UIHelper::displaySuccess("Locations info saved to 'locations_info.txt'");
                    }
                    system("pause");
                    break;
                    
                case 6:
                    // View AI predictions
                    viewAIPredictions();
                    break;
                    
                case 7:
                    // Compare route costs
                    compareRoutes();
                    break;
                    
                    case 8:
                    // Select most cost-effective route
                    system("cls");
                    displayHeader();
                    selectMostCostEffectiveRoute();
                    system("pause");
                    break;
                    
                // And update the case numbers for the following options
                case 9:  // Waste Pattern Analytics Dashboard
                    displayAnalyticsDashboard();
                    break;
                    
                    case 10:
                    // View Collection History (new option)
                    displayCollectionHistory();
                    break;
                    
                case 11:  // Help (moved from 10 to 11)
                    showHelp();
                    break;
                    
                case 12:  // Save all data (moved from 11 to 12)
                    saveData();
                    break;
                    
                case 13:  // Load data (moved from 12 to 13)
                    loadData();
                    break;
                    
                case 14:  // Delete saved data (moved from 13 to 14)
                    deleteData();
                    break;

                case 0:
                    // Exit - with confirmation and save prompt 
                    {
                        system("cls");
                        displayHeader();
        
                        if (!confirmAction("Are you sure you want to exit the system?")) {
                             break; // User changed their mind, return to menu
                        }
        
                         // Check if data needs to be saved
                         // This could be enhanced with a flag that tracks if data has changed since last save
                         if (confirmAction("Would you like to save all data before exiting?")) {
                         saveData(); // Call the existing saveData method
                    
                         }
        
                        // Display farewell message
                        system("cls");
                        displayHeader();
                         setTextColor(COLOR_GREEN);
                         cout << "\n\n";
                         cout << "  Thank You for using the Waste Collection Management System!!!\n";
                         cout << "                 Have a nice day :) !!\n\n";
                         setTextColor(COLOR_WHITE);
        
                         cout << "Press any key to exit...";
                        _getch();
        
                        running = false;
                    }   
                    break;
                    
                default:
                    cout << "Invalid choice. Please try again." << endl;
                    system("pause");
            }
        }
    }
};
/**
 * Main function that initializes the WasteManagementSystem and runs it.
 * 
 * @return 0 if the program runs successfully, otherwise 1.
 */
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

