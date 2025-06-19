// PALM_OpenFAST_Driver.cpp
#include "OpenFAST.H"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <cmath>
#include <mpi.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fstream>
#include <sys/stat.h>
#include <sstream>  // Add this at the top with other includes

// Debug macros from original code
#define debug_msg(...) {FILE *fp; \
                        fp=fopen("DEBUG_CCODE","a");\
                        fprintf(fp, __VA_ARGS__); fprintf(fp, "\n");\
                        fclose(fp); }

#define CMSGOUT 0
#define SOB 131072
#define SD 200
#define MTU 200

// Global variables for PALM communication
class PALMOpenFASTInterface {
private:
    // OpenFAST objects
    fast::OpenFAST FAST;
    fast::fastInputs fi;
    
    // Communication control variables
    int port_fast;
    std::string host_fast;
    int turbidf;
    int msgtypep;
    unsigned char msgrsp[SOB];
    int bytein_count;
    int byteout_count;
    
    // Simulation control variables
    double simtimef = 0.0;
    double dtf = 0.0;
    double endtimef = 0.0;
    int numbld;
    int numbldelem;
    float rhoairf;
    float rotrad;
    double vel_rot = 0.0;
    double ShftHtf = 0.0;
    double xs_x, xs_y, xs_z;
    int palmready = -1;
    int msgcf = 1;
    double simtimep;
    double response = 1.0;
    
    // Thread control
    std::thread server_thread;
    std::mutex response_mutex;
    std::condition_variable response_cv;
    bool procflag = false;
    bool stop_serv = false;
    int cservrun = 0;
    int server_socket;
    
    // Velocity storage
    std::vector<std::vector<double>> blade_velocities;
    std::vector<std::vector<double>> blade_positions;
    std::vector<std::vector<double>> blade_forces;
    
public:
    PALMOpenFASTInterface() {}
    ~PALMOpenFASTInterface() {
        if (server_thread.joinable()) {
            stop_serv = true;
            server_thread.join();
        }
    }
    
    // Byte conversion functions (from original C code)
    void fbconv(unsigned char *af2b, double fval, int cbyte) {
        union {
            unsigned char bytes[8];
            double double_val;
        } fbconvert;
        
        fbconvert.double_val = fval;
        for (int i = 0; i < 8; i++) {
            af2b[cbyte + i] = fbconvert.bytes[i];
        }
    }
    
    double bfconv(unsigned char *af2b, int cbyte) {
        union {
            unsigned char bytes[8];
            double double_val;
        } bfconvert;
        
        for (int i = 0; i < 8; i++) {
            bfconvert.bytes[i] = af2b[cbyte + i];
        }
        
        return bfconvert.double_val;
    }
    
    void ibconv(unsigned char *af2b, int ival, int cbyte) {
        union {
            unsigned char bytes[4];
            int int_val;
        } ibconvert;
        
        ibconvert.int_val = ival;
        for (int i = 0; i < 4; i++) {
            af2b[cbyte + i] = ibconvert.bytes[i];
        }
    }
    
    int biconv(unsigned char *af2b, int cbyte) {
        union {
            unsigned char bytes[4];
            int int_val;
        } biconvert;
        
        for (int i = 0; i < 4; i++) {
            biconvert.bytes[i] = af2b[cbyte + i];
        }
        
        return biconvert.int_val;
    }
    
    // Build message to PALM (adapted from original build_msg)
    int build_msg(unsigned char *msg2p, int msgtype) {
        int cval = 0;
        
        // Message header
        ibconv(msg2p, 66, cval);
        cval += 4;
        
        // Turbine id
        ibconv(msg2p, turbidf, cval);
        cval += 4;
        
        // Message id
        ibconv(msg2p, msgcf, cval);
        cval += 4;
        
        // Message type
        ibconv(msg2p, msgtype, cval);
        cval += 4;
        
        if (msgtype == -1) {
            std::cout << "Error occurred in FAST. Sending message to PALM." << std::endl;
            stop_serv = true;
        }
        else if (msgtype == 1) {
            // Connection check response
            fbconv(msg2p, dtf, cval);
            cval += 8;
            
            ibconv(msg2p, numbld, cval);
            cval += 4;
            
            ibconv(msg2p, numbldelem, cval);
            cval += 4;
            
            fbconv(msg2p, rotrad, cval);
            cval += 8;
            
            fbconv(msg2p, rhoairf, cval);
            cval += 8;
            
            fbconv(msg2p, ShftHtf, cval);
            cval += 8;
        }
        else if (msgtype == 2) {
            if (CMSGOUT > 0) {
                debug_msg("[Debug] FAST reached end of simulation: Stop signal sent to PALM.");
            }
            stop_serv = true;
        }
        else if (msgtype == 3 || msgtype == 4) {
            // Sending positions
            for (int i = 0; i < blade_positions.size(); i++) {
                fbconv(msg2p, blade_positions[i][0], cval);
                cval += 8;
                fbconv(msg2p, blade_positions[i][1], cval);
                cval += 8;
                fbconv(msg2p, blade_positions[i][2], cval);
                cval += 8;
            }
        }
        
        if (msgtype == 4) {
            // Sending forces
            for (int i = 0; i < blade_forces.size(); i++) {
                fbconv(msg2p, blade_forces[i][0], cval);
                cval += 8;
                fbconv(msg2p, blade_forces[i][1], cval);
                cval += 8;
                fbconv(msg2p, blade_forces[i][2], cval);
                cval += 8;
            }
            
            // Rotational speed
            fbconv(msg2p, vel_rot, cval);
            cval += 8;
            
            // Simulation time
            fbconv(msg2p, simtimef, cval);
            cval += 8;
            
            // Coordinate system shaft
            fbconv(msg2p, xs_x, cval);
            cval += 8;
            fbconv(msg2p, xs_y, cval);
            cval += 8;
            fbconv(msg2p, xs_z, cval);
            cval += 8;
        }
        else if (msgtype == 5) {
            fbconv(msg2p, response, cval);
            cval += 8;
        }
        
        // End of message
        ibconv(msg2p, 66, cval);
        cval += 4;
        
        byteout_count = cval;
        msgcf++;
        
        return 0;
    }
    
    // Analyze PALM message (adapted from original analyse_msg)
    int analyse_msg(unsigned char *velmsg) {
        int cbyte = 0;
        
        if (bytein_count < 16) {
            std::cout << "[Error] PALM message is corrupted." << std::endl;
            return -1;
        }
        
        // Start signal
        int start_signal = biconv(velmsg, cbyte);
        cbyte += 4;
        
        // Turbine id
        int turbidp = biconv(velmsg, cbyte);
        cbyte += 4;
        if (turbidf != turbidp) {
            std::cout << "[Error] Turbine id mismatch" << std::endl;
            return -1;
        }
        
        // Message id
        int msgcp = biconv(velmsg, cbyte);
        cbyte += 4;
        
        // Message type
        msgtypep = biconv(velmsg, cbyte);
        cbyte += 4;
        
        // Verify message order
        if ((msgcp != msgcf) && (msgtypep != 4)) {
            std::cout << "[Error] Message counter mismatch" << std::endl;
            return -1;
        }
        
        if (msgtypep == 1) {
            // Connection check
            std::cout << "PALM is verifying the connection." << std::endl;
            palmready = 0;
            cbyte += 20;
            simtimep = bfconv(velmsg, cbyte);
        }
        else if (msgtypep == 2) {
            // PALM is ready
            std::cout << "PALM is ready." << std::endl;
            palmready = 1;
            return 0;
        }
        else if (msgtypep == 3 || msgtypep == 5) {
            // Receiving velocities from PALM
            if (CMSGOUT > 0) {
                debug_msg("Receiving velocities from PALM...");
            }
            
            blade_velocities.clear();
            blade_velocities.resize(numbld * numbldelem + 1);
            
            for (int i = 0; i < numbld * numbldelem + 1; i++) {
                std::vector<double> vel(3);
                vel[0] = bfconv(velmsg, cbyte);
                cbyte += 8;
                vel[1] = bfconv(velmsg, cbyte);
                cbyte += 8;
                vel[2] = bfconv(velmsg, cbyte);
                cbyte += 8;
                blade_velocities[i] = vel;
            }
            return 0;
        }
        else if (msgtypep == 4) {
            // Resuming simulation
            palmready = 0;
            return 0;
        }
        else if (msgtypep == 6) {
            // PALM asking for positions
            palmready = 0;
            return 0;
        }
        
        return 0;
    }
    
    // Communication server (adapted from original commserver)
    // Communication server (adapted from original commserver)
    void commserver() {
        server_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (server_socket < 0) {
            std::cerr << "[Error] Unable to open socket." << std::endl;
            return;
        }
        
        int opt = 1;
        setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        struct sockaddr_in serv_addr;
        memset(&serv_addr, 0, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        serv_addr.sin_port = htons(port_fast);
        
        if (bind(server_socket, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            std::cerr << "[Error] Unable to bind." << std::endl;
            close(server_socket);
            return;
        }
        
        listen(server_socket, 5);
        stop_serv = false;
        
        while (!stop_serv) {
            if (CMSGOUT > 0) {
                std::cout << "Waiting for new connection..." << std::endl;
            }
            
            struct sockaddr_in cli_addr;
            socklen_t clilen = sizeof(cli_addr);
            int newsockfd = accept(server_socket, (struct sockaddr*)&cli_addr, &clilen);
            
            if (newsockfd < 0) {
                std::cerr << "[Error] Unable to accept connection." << std::endl;
                continue;
            }
            
            // Read message length
            unsigned char msgrsppack[MTU + 1];
            int n = read(newsockfd, msgrsppack, 4);
            if (n != 4) {
                std::cerr << "[Error] Unable to read from socket." << std::endl;
                close(newsockfd);
                continue;
            }
            
            int imsglen = biconv(msgrsppack, 0);
            
            // Read data
            memset(msgrsp, 0, SOB);
            int bytesread = 0;
            while (bytesread < imsglen) {
                n = read(newsockfd, msgrsppack, MTU);
                if (n < 0) {
                    std::cerr << "[Error] Unable to read from socket." << std::endl;
                    close(newsockfd);
                    continue;
                }
                memcpy((msgrsp + bytesread), msgrsppack, n);
                bytesread += n;
            }
            
            bytein_count = bytesread;
            
            // Analyze received message
            int cflag = analyse_msg(msgrsp);
            if (cflag != 0) {
                std::cerr << "[Error] Corrupted message." << std::endl;
                close(newsockfd);
                continue;
            }
            
            // Build response
            unsigned char msgout[SOB + 1];
            if (endtimef - simtimef <= 0.5 * dtf) {
                cflag = build_msg(msgout, 2);
            } else {
                if (msgtypep == 1 || msgtypep == 4) {
                    cflag = build_msg(msgout, 1);
                } else if (msgtypep == 2 || msgtypep == 6) {
                    cflag = build_msg(msgout, 3);
                } else if (msgtypep == 3) {
                    // Allow FAST to proceed
                    {
                        std::lock_guard<std::mutex> lock(response_mutex);
                        procflag = true;
                    }
                    response_cv.notify_all();
                    
                    // Wait for FAST to finish time step
                    {
                        std::unique_lock<std::mutex> lock(response_mutex);
                        response_cv.wait(lock, [this]{ return !procflag; });
                    }
                    
                    cflag = build_msg(msgout, 4);
                } else if (msgtypep == 5) {
                    cflag = build_msg(msgout, 5);
                }
            }
            
            // Send response
            if (cflag > -1) {
                unsigned char omsglen[4];
                ibconv(omsglen, byteout_count, 0);
                
                cflag = write(newsockfd, omsglen, 4);
                if (cflag < 0) {
                    std::cerr << "[Error] Writing to socket" << std::endl;
                    close(newsockfd);
                    continue;
                }
                
                cflag = write(newsockfd, msgout, byteout_count);
                if (cflag < 0) {
                    std::cerr << "[Error] Writing to socket" << std::endl;
                    close(newsockfd);
                    continue;
                }
            }
            
            close(newsockfd);
        }
        
        close(server_socket);
        
        // Allow FAST to proceed if waiting
        {
            std::lock_guard<std::mutex> lock(response_mutex);
            procflag = true;
        }
        response_cv.notify_all();
    }
    
    // Read communication data file (.srv file)
    int read_commdata(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "[Error] Unable to open communication data file: " << filename << std::endl;
            return -1;
        }
        
        std::string line, dummy;
        int lineCount = 0;
        
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            if (lineCount == 0) {
                iss >> dummy >> turbidf;
                if (dummy != "TURBINE_ID") {
                    std::cerr << "[Error] Invalid format in .srv file" << std::endl;
                    return -1;
                }
            } else if (lineCount == 1) {
                iss >> dummy >> host_fast;
                if (dummy != "HOST_FAST") {
                    std::cerr << "[Error] Invalid format in .srv file" << std::endl;
                    return -1;
                }
            } else if (lineCount == 2) {
                iss >> dummy >> port_fast;
                if (dummy != "PORT_FAST") {
                    std::cerr << "[Error] Invalid format in .srv file" << std::endl;
                    return -1;
                }
            }
            lineCount++;
        }
        
        if (lineCount != 3) {
            std::cerr << "[Error] Communication data file is corrupted" << std::endl;
            return -1;
        }
        
        return 0;
    }
    
    // Update blade positions from OpenFAST
    void updateBladePositions() {
        blade_positions.clear();
        int iTurb = 0; // Assuming single turbine for now
        
        // Get all force nodes (blade + tower)
        int nForcePts = FAST.get_numForcePts(iTurb);
        
        for (int i = 0; i < nForcePts; i++) {
            std::vector<double> coords(3);
            FAST.getForceNodeCoordinates(coords, i, iTurb, fast::STATE_NP1);
            blade_positions.push_back(coords);
        }
    }
    
    // Update blade forces from OpenFAST
    void updateBladeForces() {
        blade_forces.clear();
        int iTurb = 0; // Assuming single turbine for now
        
        int nForcePts = FAST.get_numForcePts(iTurb);
        
        for (int i = 0; i < nForcePts; i++) {
            std::vector<double> force(3);
            FAST.getForce(force, i, iTurb, fast::STATE_NP1);
            blade_forces.push_back(force);
        }
    }
    
    // Apply velocities from PALM to OpenFAST
    void applyVelocitiesToOpenFAST() {
        if (blade_velocities.empty()) return;
        
        int iTurb = 0; // Assuming single turbine for now
        int nVelPts = FAST.get_numVelPts(iTurb);
        
        // Apply velocities to velocity nodes
        for (int i = 0; i < nVelPts && i < blade_velocities.size(); i++) {
            FAST.setVelocity(blade_velocities[i], i, iTurb, fast::STATE_NP1);
        }
    }
    
    // Get turbine properties from OpenFAST
    void getTurbineProperties() {
        int iTurb = 0;
        
        // Get timestep
        dtf = FAST.get_timestep();
        
        // Get turbine properties from the OpenFAST turbine data
        // These would need to be extracted from OpenFAST's internal data structures
        // For now, using default values
        numbld = 3;  // Typical 3-blade turbine
        numbldelem = FAST.get_numForcePts(iTurb) / numbld;  // Estimate
        
        // Get hub properties
        std::vector<double> hubPos(3);
        std::vector<double> hubVel(6);
        FAST.getHubPos(hubPos, iTurb, fast::STATE_N);
        FAST.getHubVel(hubVel, iTurb, fast::STATE_N);
        
        // Calculate rotor radius (would need proper extraction from OpenFAST)
        rotrad = 63.0;  // Default value, should be extracted from turbine data
        ShftHtf = hubPos[2];  // Z-coordinate of hub
        
        // Get air density from turbine data
        rhoairf = 1.225;  // Default air density
        
        // Get rotational velocity
        vel_rot = hubVel[5];  // Rotational velocity around z-axis
    }
    
public:
    // Main interface function to run the coupled simulation
    int runCoupledSimulation(const std::string& openfastInput, const std::string& palmCommFile, 
                            double tStart, double tEnd) {
        
        // Read PALM communication data
        if (read_commdata(palmCommFile) != 0) {
            return -1;
        }
        
        // Set up OpenFAST inputs
        fi.comm = MPI_COMM_WORLD;
        fi.nTurbinesGlob = 1;
        fi.dryRun = false;
        fi.debug = false;
        fi.simStart = fast::init;
        fi.restartFreq = 1000;
        fi.outputFreq = 100;
        fi.dtDriver = 0.01;  // This should be set appropriately
        fi.tMax = tEnd;
        
        // Set up turbine data
        fi.globTurbineData.resize(1);
        fi.globTurbineData[0].TurbID = turbidf;
        fi.globTurbineData[0].sType = fast::EXTINFLOW;
        fi.globTurbineData[0].FASTInputFileName = openfastInput;
        fi.globTurbineData[0].TurbineBasePos = {0.0, 0.0, 0.0};
        fi.globTurbineData[0].TurbineHubPos = {0.0, 0.0, 90.0};  // Default hub height
        fi.globTurbineData[0].numForcePtsBlade = 20;  // Default
        fi.globTurbineData[0].numForcePtsTwr = 20;    // Default
        
        // Initialize OpenFAST
        FAST.setInputs(fi);
        FAST.allocateTurbinesToProcsSimple();
        
        // Start communication server thread
        cservrun = 1;
        server_thread = std::thread(&PALMOpenFASTInterface::commserver, this);
        
        // Initialize OpenFAST
        FAST.init();
        
        // Get turbine properties after initialization
        getTurbineProperties();
        endtimef = tEnd;
        simtimef = tStart;
        
        // Handle initial solution if needed
        if (FAST.isTimeZero()) {
            FAST.solution0();
        }
        
        // Calculate time steps
        int ntStart = int(tStart / fi.dtDriver);
        int ntEnd = int(tEnd / fi.dtDriver);
        int nSubsteps = int(fi.dtDriver / FAST.get_timestep());
        
        // Main simulation loop
        for (int nt = ntStart; nt < ntEnd; nt++) {
            simtimef = nt * fi.dtDriver;
            
            // Wait for PALM signal if needed
            if (msgtypep == 3) {
                std::unique_lock<std::mutex> lock(response_mutex);
                response_cv.wait(lock, [this]{ return procflag || stop_serv; });
                
                if (stop_serv) break;
            }
            
            // Apply velocities from PALM
            applyVelocitiesToOpenFAST();
            
            // Run substeps
            for (int iSubstep = 0; iSubstep < nSubsteps; iSubstep++) {
                FAST.step();
            }
            
            // Update positions and forces
            updateBladePositions();
            updateBladeForces();
            
            // Get updated turbine state
            int iTurb = 0;
            std::vector<double> hubVel(6);
            FAST.getHubVel(hubVel, iTurb, fast::STATE_NP1);
            vel_rot = hubVel[5];
            
            // Get shaft orientation (simplified - should get from OpenFAST)
            xs_x = 1.0;
            xs_y = 0.0;
            xs_z = 0.0;
            
            // Signal completion if waiting
            if (msgtypep == 3) {
                std::lock_guard<std::mutex> lock(response_mutex);
                procflag = false;
            }
            response_cv.notify_all();
        }
        
        // Clean up
        stop_serv = true;
        if (server_thread.joinable()) {
            server_thread.join();
        }
        
        FAST.end();
        
        return 0;
    }
};

// Helper functions for reading input
inline bool checkFileExists(const std::string& name) {
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

template<typename T>
void get_if_present(const YAML::Node& node, const std::string& key, T& result, 
                   const T& default_if_not_present = T()) {
    if (node[key]) {
        result = node[key].as<T>();
    } else {
        result = default_if_not_present;
    }
}

template<typename T>
void get_required(const YAML::Node& node, const std::string& key, T& result) {
    if (node[key]) {
        result = node[key].as<T>();
    } else {
        throw std::runtime_error("Error: parsing missing required key: " + key);
    }
}

// Main function
int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Incorrect syntax. Try: palm_openfast inputfile.yaml" << std::endl;
        return 1;
    }
    
    int iErr;
    int nProcs;
    int rank;
    
    iErr = MPI_Init(NULL, NULL);
    iErr = MPI_Comm_size(MPI_COMM_WORLD, &nProcs);
    iErr = MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    
    try {
        // Read YAML configuration
                // Read YAML configuration
        std::string configFile = argv[1];
        if (!checkFileExists(configFile)) {
            throw std::runtime_error("Input file " + configFile + " does not exist");
        }
        
        YAML::Node config = YAML::LoadFile(configFile);
        
        // Read configuration parameters
        std::string openfastInput;
        std::string palmCommFile;
        double tStart, tEnd;
        
        get_required(config, "openfast_input", openfastInput);
        get_required(config, "palm_comm_file", palmCommFile);
        get_required(config, "t_start", tStart);
        get_required(config, "t_end", tEnd);
        
        // Create and run the interface
        PALMOpenFASTInterface interface;
        int result = interface.runCoupledSimulation(openfastInput, palmCommFile, tStart, tEnd);
        
        MPI_Finalize();
        return result;
        
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        std::cerr << "Program quitting now" << std::endl;
        MPI_Finalize();
        return 1;
    }
}
                