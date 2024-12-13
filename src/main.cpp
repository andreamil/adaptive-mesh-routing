#include "MeshNode.h"

const int MESH_PORT = 5555;
const char* MESH_PREFIX = "MeshNet";
const char* MESH_PASSWORD = "MeshPass123";

painlessMesh mesh;
MeshNode* node = nullptr;

void receivedCallback(uint32_t from, String &msg) {
    if (node) {
        node->handleCommand(msg);
    }
}

void newConnectionCallback(uint32_t nodeId) {
    if (node) {
        node->update();
    }
}

void changedConnectionCallback() {
    if (node) {
        node->update();
    }
}

void nodeTimeAdjustedCallback(int32_t offset) {
    // Required for painlessMesh
}

void handleSerialCommand(const String& inputCmd) {
    String command = inputCmd;
    command.trim();

    if (command == SerialCommands::CMD_HELP) {
        Serial.println(SerialCommands::CMD_FORMAT);
        return;
    }

    if (command == SerialCommands::CMD_CANCEL_ROUTE) {
        // if (node) {
        //     node->cancelRoute();
        // }
        
        auto msg = MeshCommands::createRouteRequest(0, 0);
        Serial.print("Routing from ");
        Serial.print(0);
        Serial.print(" to ");
        Serial.println(0);
        mesh.sendBroadcast(MeshCommands::serializeMessage(msg), true);
        return;
    }

    int firstSpace = command.indexOf(' ');
    if (firstSpace == -1) {
        if (command == SerialCommands::CMD_TOPOLOGY) {
            Serial.println(node->getTopology());
        }
        else if (command == SerialCommands::CMD_NODES) {
            Serial.println(node->getTopology());
        }
        else if (command == SerialCommands::CMD_RESET) {
            MeshCommands::Message msg(CommandType::RESET_CONFIG);
            mesh.sendBroadcast(MeshCommands::serializeMessage(msg), true);
        }
        return;
    }

    String cmd = command.substring(0, firstSpace);
    String params = command.substring(firstSpace + 1);

    if (cmd == SerialCommands::CMD_UPDATE_RSSI) {
        int firstSpace = params.indexOf(' ');
        if (firstSpace != -1) {
            String restParams = params.substring(firstSpace + 1);
            int secondSpace = restParams.indexOf(' ');
            if (secondSpace != -1) {
                uint32_t fromNode = strtoul(params.substring(0, firstSpace).c_str(), nullptr, 10);
                uint32_t toNode = strtoul(restParams.substring(0, secondSpace).c_str(), nullptr, 10);
                int32_t rssiValue = restParams.substring(secondSpace + 1).toInt();
                
                if (node) {
                    node->updateRSSI(fromNode, toNode, rssiValue);
                }
                
                auto msg = MeshCommands::createRSSIUpdate(fromNode, toNode, rssiValue);
                mesh.sendBroadcast(MeshCommands::serializeMessage(msg), true);
                
                Serial.print(F("Updated RSSI: from "));
                Serial.print(fromNode);
                Serial.print(F(" to "));
                Serial.print(toNode);
                Serial.print(F(" = "));
                Serial.println(rssiValue);
            }
        }
    }

    else if (cmd == SerialCommands::CMD_ROUTE) {
        int secondSpace = params.indexOf(' ');
        if (secondSpace != -1) {
            uint32_t src = strtoul(params.substring(0, secondSpace).c_str(), NULL, 10);
            uint32_t dest = strtoul(params.substring(secondSpace + 1).c_str(), NULL, 10);
            auto msg = MeshCommands::createRouteRequest(src, dest);
            Serial.print("Routing from ");
            Serial.print(src);
            Serial.print(" to ");
            Serial.println(dest);
            mesh.sendBroadcast(MeshCommands::serializeMessage(msg), true);
        }
    }
    else if (cmd == SerialCommands::CMD_ENABLE || cmd == SerialCommands::CMD_DISABLE) {
        String target = params;
        uint32_t targetNode = (target == "all") ? 0 : strtoul(target.c_str(), nullptr, 10);
        auto msg = MeshCommands::createNodeControl(targetNode, cmd == SerialCommands::CMD_ENABLE);
        mesh.sendBroadcast(MeshCommands::serializeMessage(msg), true);
    }
    else if (cmd == SerialCommands::CMD_RSSI_THRESHOLD) {
        int secondSpace = params.indexOf(' ');
        if (secondSpace != -1) {
            String target = params.substring(0, secondSpace);
            int32_t threshold = params.substring(secondSpace + 1).toInt();
            uint32_t targetNode = (target == "all") ? 0 : strtoul(target.c_str(), nullptr, 10);
            auto msg = MeshCommands::createRSSIThresholdConfig(targetNode, threshold);
            mesh.sendBroadcast(MeshCommands::serializeMessage(msg), true);
        }
    }
    else if (cmd == SerialCommands::CMD_EXCLUDE) {
        int secondSpace = params.indexOf(' ');
        if (secondSpace != -1) {
            uint32_t node1 = strtoul(params.substring(0, secondSpace).c_str(), nullptr, 10);
            uint32_t node2 = strtoul(params.substring(secondSpace + 1).c_str(), nullptr, 10);
            auto msg = MeshCommands::createPathExclusion(node1, node2);
            mesh.sendBroadcast(MeshCommands::serializeMessage(msg), true);
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        ; // Wait for Serial to be ready
    }

    mesh.setDebugMsgTypes(ERROR | STARTUP);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT);
    mesh.onReceive(&receivedCallback);
    mesh.onNewConnection(&newConnectionCallback);
    mesh.onChangedConnections(&changedConnectionCallback);
    mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

    node = new MeshNode(mesh);

    Serial.println(F("\nMesh Node Started"));
    Serial.println(F("Type 'help' for commands"));
}

void loop() {
    mesh.update();
    
    if (node){
        if (node->nodeId != 0) {
            node->update();

            if (Serial.available()) {
                node->updateNow();
                String command = Serial.readStringUntil('\n');
                handleSerialCommand(command);
                node->updateNow();
            }
        }
        node->updateLEDs();
    }
}