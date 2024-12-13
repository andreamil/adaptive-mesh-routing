#pragma once

#include "MeshTypes.h"
#include "MeshCommands.h"
#include <TaskSchedulerDeclarations.h>

class MeshNode {
public:
    explicit MeshNode(painlessMesh& mesh);
    
    void update();
    void updateNow();
    void handleCommand(const String& command);
    String getTopology();
    bool isNodeValid(uint32_t nodeId, int32_t rssi);
    void cancelRoute();
    void updateRSSI(uint32_t fromNodeId, uint32_t toNodeId, int32_t value);
    void updateLEDs();
    uint32_t nodeId;
    
private:
    painlessMesh& mesh;
    std::map<uint32_t, NodeInfo> nodes;
    RSSIConfig globalRssiConfig;
    std::set<std::pair<uint32_t, uint32_t>> excludedPaths;
    std::map<uint32_t, int32_t> rssiValues;
    bool ledState;
    unsigned long lastLedUpdate;
    unsigned long lastTopologyUpdate;

    void handleRouteRequest(const MeshCommands::Message& msg);
    void handleNodeControl(const MeshCommands::Message& msg);
    void handleRSSIConfig(const MeshCommands::Message& msg);
    void handleRSSIThresholdConfig(const MeshCommands::Message& msg);
    void handlePathExclusion(const MeshCommands::Message& msg);
    void updateNodeStates();
    void broadcastNodeState();
    void broadcastTopology();
    RouteInfo calculateRoute(uint32_t src, uint32_t dest);
    float calculatePathWeight(uint32_t from, uint32_t to);
    int32_t getRSSI(uint32_t nodeId);

    struct ActiveRoute {
        uint32_t sourceNode;
        uint32_t destNode;
        bool active;
        unsigned long lastUpdate;
        
        ActiveRoute() : 
            sourceNode(0), 
            destNode(0), 
            active(false), 
            lastUpdate(0) {}
    } activeRoute;

    void updateActiveRoute();
    void cancelActiveRoute();
};