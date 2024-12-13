#pragma once

namespace SerialCommands {
    const char* const CMD_HELP = "help";
    const char* const CMD_ROUTE = "route";
    const char* const CMD_CANCEL_ROUTE = "cancel";
    const char* const CMD_ENABLE = "enable";
    const char* const CMD_DISABLE = "disable";
    const char* const CMD_RSSI_THRESHOLD = "rssi";
    const char* const CMD_EXCLUDE = "exclude";
    const char* const CMD_TOPOLOGY = "topology";
    const char* const CMD_NODES = "nodes";
    const char* const CMD_RESET = "reset";
    const char* const CMD_UPDATE_RSSI = "update_rssi"; // New command

    const char* const CMD_FORMAT = R"(
Available Commands:
help                                      - Show this help
route <srcId> <destId>                   - Show route between nodes
cancel                                   - Cancel active route
enable <nodeId|all>                      - Enable node(s)
disable <nodeId|all>                     - Disable node(s)
rssi <nodeId|all> <threshold>            - Set RSSI threshold for node(s)
update_rssi <fromNode> <toNode> <value>  - Update RSSI value for connection between nodes
exclude <nodeId> <nodeId>                - Exclude path between nodes
topology                                 - Show network topology
nodes                                    - List connected nodes
reset                                    - Reset all configurations
)";
}

namespace MeshCommands {
    struct Message {
        CommandType type;
        JsonDocument data;

        Message(CommandType t) : 
            type(t), 
            data() {
            data["type"] = static_cast<int>(t);
        }
    };

    inline Message createRouteRequest(uint32_t src, uint32_t dest) {
        Message msg(CommandType::ROUTE_REQUEST);
        msg.data["src"] = src;
        msg.data["dest"] = dest;
        return msg;
    }

    inline Message createNodeControl(uint32_t nodeId, bool enable) {
        Message msg(CommandType::NODE_CONTROL);
        msg.data["node"] = nodeId;
        msg.data["enable"] = enable;
        return msg;
    }

    inline Message createRSSIThresholdConfig(uint32_t nodeId, int32_t threshold) {
        Message msg(CommandType::RSSI_THRESHOLD_CONFIG);
        msg.data["node"] = nodeId;
        msg.data["threshold"] = threshold;
        return msg;
    }

    inline Message createRSSIUpdate(uint32_t fromNode, uint32_t toNode, int32_t value) {
        Message msg(CommandType::RSSI_CONFIG);
        msg.data["from"] = fromNode;
        msg.data["to"] = toNode;
        msg.data["value"] = value;
        return msg;
    }

    inline Message createPathExclusion(uint32_t node1, uint32_t node2) {
        Message msg(CommandType::PATH_EXCLUDE);
        msg.data["node1"] = node1;
        msg.data["node2"] = node2;
        return msg;
    }

    inline Message createPathUpdate(const std::vector<uint32_t>& path) {
        Message msg(CommandType::PATH_UPDATE);
        JsonArray pathArray = msg.data["path"].to<JsonArray>();
        for (auto nodeId : path) {
            pathArray.add(nodeId);
        }
        return msg;
    }

    inline String serializeMessage(const Message& msg) {
        String result;
        serializeJson(msg.data, result);
        return result;
    }

    inline Message parseMessage(const String& json) {
        JsonDocument doc; 
        DeserializationError error = deserializeJson(doc, json);
        if (error) {
            Serial.print(F("JSON parsing failed: "));
            Serial.println(error.c_str());
            return Message(CommandType::RESET_CONFIG);
        }
        CommandType type = static_cast<CommandType>(doc["type"].as<int>());
        Message msg(type);
        msg.data = doc;
        return msg;
    }
}