#pragma once 

#include <vector>
#include <map>
#include <set>
#include <queue>
#include <Arduino.h>
#include <painlessMesh.h>
#include <ArduinoJson.h>

// Hardware definitions
#ifdef ESP32
    #define LED_PIN 2
#else
    #define LED_PIN LED_BUILTIN
    #define HIGH 0 // No ESP8266, HIGH apaga o LED e LOW acende
    #define LOW 1 
#endif

enum class CommandType {
    ROUTE_REQUEST,     
    NODE_CONTROL,      
    RSSI_CONFIG,          
    PATH_EXCLUDE,      
    TOPOLOGY_REQUEST,  
    NODE_LIST,         
    PATH_UPDATE,       
    RESET_CONFIG,      
    TOPOLOGY_UPDATE,   
    NODE_STATE,         
    RSSI_THRESHOLD_CONFIG
};

struct NodeInfo {
    uint32_t id;
    bool active;
    bool onPath;
    int32_t rssiThreshold;  // RSSI threshold individual do nó
    std::map<uint32_t, int32_t> connections;  // Mapa de conexões com RSSI

    NodeInfo() : 
        id(0), 
        active(true), 
        onPath(false),
        rssiThreshold(-100) {}  // Threshold padrão
};

struct RSSIConfig { 
    bool enabled;           
    std::set<uint32_t> excludedNodes;  

    RSSIConfig() :   
        enabled(true) {}
};

struct RouteInfo {
    std::vector<uint32_t> path;
    float totalWeight;
    bool valid;

    RouteInfo() : 
        totalWeight(0), 
        valid(false) {}
};