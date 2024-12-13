#include "MeshNode.h"

const unsigned long LED_BLINK_INTERVAL = 500;
const unsigned long TOPOLOGY_UPDATE_INTERVAL = 1000;
const unsigned long ROUTE_UPDATE_INTERVAL = 1000;

MeshNode::MeshNode(painlessMesh& _mesh) : 
    mesh(_mesh),
    ledState(false),
    lastLedUpdate(0),
    lastTopologyUpdate(0) {
    
    nodeId = mesh.getNodeId();
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    nodes[nodeId] = NodeInfo();
    nodes[nodeId].id = nodeId;
    nodes[nodeId].active = true;
}
    
void MeshNode::updateNow() {
    updateNodeStates();
    updateActiveRoute();
}

void MeshNode::update() {
    unsigned long now = millis();  
    if (now - lastTopologyUpdate >= TOPOLOGY_UPDATE_INTERVAL) {
        updateNodeStates();
        lastTopologyUpdate = now;
        updateActiveRoute();
    }
}

void MeshNode::handleCommand(const String& command) {
    auto msg = MeshCommands::parseMessage(command);

    Serial.println("Received command: " + command);

    switch (msg.type) {
        case CommandType::ROUTE_REQUEST:
            handleRouteRequest(msg);
            break;
        case CommandType::NODE_CONTROL:
            handleNodeControl(msg);
            break;
        case CommandType::RSSI_THRESHOLD_CONFIG:
            handleRSSIThresholdConfig(msg);
            break;
        case CommandType::RSSI_CONFIG:
            handleRSSIConfig(msg);
            break;
        case CommandType::PATH_EXCLUDE:
            handlePathExclusion(msg);
            break;
        case CommandType::TOPOLOGY_REQUEST:
            Serial.println(getTopology());
            break;
        case CommandType::RESET_CONFIG:
            globalRssiConfig = RSSIConfig();
            excludedPaths.clear();
            for (auto& node : nodes) {
                node.second.connections = std::map<uint32_t, int32_t>();
            }
            break;
        case CommandType::TOPOLOGY_UPDATE:
            {
                uint32_t senderId = msg.data["senderId"];
                if (msg.data["connections"].is<JsonObject>()) {
                    JsonObject conns = msg.data["connections"].as<JsonObject>();
                    nodes[senderId].connections.clear();
                    for (JsonPair kv : conns) {
                        uint32_t connectedNodeId = strtoul(kv.key().c_str(), NULL, 10);
                        nodes[senderId].connections[connectedNodeId] = kv.value().as<int32_t>();
                    }
                }
            }
            break;
        case CommandType::NODE_STATE:
            {
                uint32_t senderId = msg.data["id"];
                nodes[senderId].active = msg.data["active"];
                nodes[senderId].rssiThreshold = msg.data["rssiThreshold"];
                
                if (msg.data["rssiValues"].is<JsonObject>()) {
                    JsonObject rssiValues = msg.data["rssiValues"].as<JsonObject>();
                    for (JsonPair kv : rssiValues) {
                        uint32_t connectedNodeId = strtoul(kv.key().c_str(), NULL, 10);
                        nodes[senderId].connections[connectedNodeId] = kv.value().as<int32_t>();
                    }
                }
            }
            break;
        case CommandType::PATH_UPDATE:
            {
                for (auto& node : nodes) {
                    node.second.onPath = false;
                }
                JsonArray path = msg.data["path"].as<JsonArray>();
                for (JsonVariant v : path) {
                    uint32_t pathNodeId = v.as<uint32_t>();
                    if (nodes.count(pathNodeId)) {
                        nodes[pathNodeId].onPath = true;
                    }
                }
            }
            break;
        default:
            break;
    }
}

void MeshNode::handleRouteRequest(const MeshCommands::Message& msg) {
    uint32_t src = msg.data["src"];
    uint32_t dest = msg.data["dest"];

    activeRoute.sourceNode = src;
    activeRoute.destNode = dest;
    activeRoute.active = true;
    activeRoute.lastUpdate = millis();

    auto route = calculateRoute(src, dest);
    if (route.valid) {

        auto pathMsg = MeshCommands::createPathUpdate(route.path);
        mesh.sendBroadcast(MeshCommands::serializeMessage(pathMsg), true);

        for (auto& node : nodes) {
            node.second.onPath = false;
        }
        for (auto pathNode : route.path) {
            if (nodes.count(pathNode)) {
                nodes[pathNode].onPath = true;
            }
        }

        Serial.println("Route established");
    } else {
        Serial.println(F("Route not found"));
        cancelActiveRoute();
    }
}

void MeshNode::updateActiveRoute() {
    // if (!activeRoute.active) return;

    auto route = calculateRoute(activeRoute.sourceNode, activeRoute.destNode);
    if (route.valid) {
        MeshCommands::Message pathMsg(CommandType::PATH_UPDATE);
        JsonArray path = pathMsg.data["path"].to<JsonArray>();
        for (auto nodeId : route.path) {
            path.add(nodeId);
        }

        mesh.sendBroadcast(MeshCommands::serializeMessage(pathMsg));

        for (auto& node : nodes) {
            node.second.onPath = false;
        }
        for (auto pathNode : route.path) {
            if (nodes.count(pathNode)) {
                nodes[pathNode].onPath = true;
            }
        }
    } else {
        Serial.println(F("Route lost"));
        cancelActiveRoute();
    }
}

void MeshNode::cancelActiveRoute() {
    // activeRoute.active = false;
    
    for (auto& node : nodes) {
        node.second.onPath = false;
    }

    MeshCommands::Message pathMsg(CommandType::PATH_UPDATE);
    // JsonArray path = pathMsg.data["path"].to<JsonArray>();
    mesh.sendBroadcast(MeshCommands::serializeMessage(pathMsg));
    
    Serial.println(F("Route cancelled"));
}

void MeshNode::cancelRoute() {
    cancelActiveRoute();
}

void MeshNode::handleNodeControl(const MeshCommands::Message& msg) {
    uint32_t targetNode = msg.data["node"];
    bool enable = msg.data["enable"];

    if (targetNode == 0) {
        for (auto& node : nodes) {
            node.second.active = enable;
        }
    } else if (nodes.count(targetNode)) {
        nodes[targetNode].active = enable;
    }

    broadcastNodeState();
}

void MeshNode::handleRSSIThresholdConfig(const MeshCommands::Message& msg) {
    uint32_t targetNode = msg.data["node"];
    int32_t threshold = msg.data["threshold"];
    if (targetNode == 0) {
        for (auto& node : nodes) {
            node.second.rssiThreshold = threshold;
        }
    } else if (nodes.count(targetNode)) {
        nodes[targetNode].rssiThreshold = threshold;
    }
}

void MeshNode::handleRSSIConfig(const MeshCommands::Message& msg) {
    uint32_t fromNode = msg.data["from"].as<uint32_t>();
    uint32_t toNode = msg.data["to"].as<uint32_t>();
    int32_t rssiValue = msg.data["value"].as<int32_t>();
    updateRSSI(fromNode, toNode, rssiValue);
}

void MeshNode::handlePathExclusion(const MeshCommands::Message& msg) {
    uint32_t node1 = msg.data["node1"];
    uint32_t node2 = msg.data["node2"];
    excludedPaths.insert(std::minmax(node1, node2));
}

void MeshNode::updateNodeStates() {
    auto connectedNodes = mesh.getNodeList();
    
    // Atualiza conexões para nós existentes
    for (auto nodeId : connectedNodes) {
        if (nodes.count(nodeId) == 0) {
            nodes[nodeId] = NodeInfo();
            nodes[nodeId].id = nodeId;
        }
        nodes[nodeId].active = true;
        
        // Atualiza o RSSI real da conexão (poderia vir do hardware)
        int32_t currentRssi = getRSSI(nodeId);
        nodes[nodeId].connections[this->nodeId] = currentRssi;
        nodes[this->nodeId].connections[nodeId] = currentRssi;
    }

    // Remove nós que não estão mais conectados
    for (auto it = nodes.begin(); it != nodes.end();) {
        if (it->first != nodeId && 
            std::find(connectedNodes.begin(), connectedNodes.end(), it->first) == connectedNodes.end()) {
            it = nodes.erase(it);
        } else {
            ++it;
        }
    }

    // Limpa RSSIs de conexões que não existem mais
    for (auto& node : nodes) {
        std::set<uint32_t> invalidConnections;
        for (const auto& conn : node.second.connections) {
            if (nodes.count(conn.first) == 0) {
                invalidConnections.insert(conn.first);
            }
        }
        for (auto connId : invalidConnections) {
            node.second.connections.erase(connId);
        }
    }
    broadcastTopology();
}

int32_t MeshNode::getRSSI(uint32_t targetNodeId) {
    if (nodes.count(targetNodeId) && nodes[targetNodeId].connections.count(nodeId)) {
        return nodes[targetNodeId].connections[nodeId];
    }
    return -75; // Default RSSI value
}

void MeshNode::updateRSSI(uint32_t fromNodeId, uint32_t toNodeId, int32_t value) {
    if (nodes.count(fromNodeId)) {
        nodes[fromNodeId].connections[toNodeId] = value;
    }
    if (nodes.count(toNodeId)) {
        nodes[toNodeId].connections[fromNodeId] = value;
        // broadcastNodeState();
    }

    // Serial.println("Nodes:");
    // for (const auto& node : nodes) {
    //     Serial.print("Node ID: ");
    //     Serial.println(node.first);
    //     Serial.print("Active: ");
    //     Serial.println(node.second.active);
    //     Serial.print("RSSI Threshold: ");
    //     Serial.println(node.second.rssiThreshold);
    //     Serial.print("On Path: ");
    //     Serial.println(node.second.onPath);
    //     Serial.println("Connections:");
    //     for (const auto& conn : node.second.connections) {
    //         Serial.print("  Connected Node ID: ");
    //         Serial.println(conn.first);
    //         Serial.print("  RSSI: ");
    //         Serial.println(conn.second);
    //     }
    // }
}

bool MeshNode::isNodeValid(uint32_t nodeId, int32_t rssi) {
    if (!nodes.count(nodeId) || !nodes[nodeId].active) {
        return false;
    }

    // Verifica se o RSSI está acima do threshold do nó de destino
    if (rssi < nodes[nodeId].rssiThreshold) {
        return false;
    }

    return true;
}

RouteInfo MeshNode::calculateRoute(uint32_t src, uint32_t dest) {
    RouteInfo route;
    std::map<uint32_t, float> dist;
    std::map<uint32_t, uint32_t> prev;
    std::set<uint32_t> unvisited;

    // Inicializa distâncias
    for (const auto& node : nodes) {
        dist[node.first] = INFINITY;
        unvisited.insert(node.first);
    }
    dist[src] = 0;

    while (!unvisited.empty()) {
        uint32_t current = *std::min_element(
            unvisited.begin(), unvisited.end(),
            [&dist](uint32_t a, uint32_t b) {
                return dist[a] < dist[b];
            }
        );

        if (current == dest) break;
        if (dist[current] == INFINITY) break;

        unvisited.erase(current);

        // Verifica todas as conexões do nó atual
        for (auto next : nodes[current].connections) {
            if (unvisited.count(next.first)) {
                // Verifica se o caminho está excluído
                auto path = std::minmax(current, next.first);
                if (excludedPaths.count(path)) continue;

                // Verifica se o nó está ativo e se o RSSI está acima do threshold
                if (!nodes[next.first].active) continue;

                // Obtém o RSSI da conexão entre os nós
                int32_t rssi = -100;
                if (nodes[current].connections.count(next.first)) {
                    rssi = nodes[current].connections[next.first];
                }

                // Verifica se o RSSI está acima do threshold do nó de destino
                if (rssi < nodes[next.first].rssiThreshold || rssi < nodes[current].rssiThreshold) continue;

                float weight = calculatePathWeight(current, next.first);
                float alt = dist[current] + weight;

                if (alt < dist[next.first]) {
                    dist[next.first] = alt;
                    prev[next.first] = current;
                }
            }
        }
    }

    // Reconstrói o caminho se um foi encontrado
    if (dist[dest] != INFINITY) {
        uint32_t current = dest;
        while (current != src) {
            route.path.insert(route.path.begin(), current);
            current = prev[current];
        }
        route.path.insert(route.path.begin(), src);
        route.totalWeight = dist[dest];
        route.valid = true;
    }

    return route;
}

float MeshNode::calculatePathWeight(uint32_t from, uint32_t to) {
    float weight = 1.0;

    // Obtém o RSSI da conexão
    int32_t rssi = -100;
    if (nodes[from].connections.count(to)) {
        rssi = nodes[from].connections[to];
    }

    // Normaliza o RSSI para um peso entre 0 e 1
    // -30 dBm (melhor caso) -> peso menor
    // -100 dBm (pior caso) -> peso maior
    float rssiWeight = (abs(rssi) - 30) / 70.0;
    rssiWeight = std::max(0.0f, std::min(1.0f, rssiWeight));

    // Considera o número de conexões do nó de destino como fator de congestionamento
    size_t connections = nodes[to].connections.size();
    float congestionWeight = static_cast<float>(connections) / 8.0;
    congestionWeight = std::min(1.0f, congestionWeight);

    // Pesos relativos para RSSI e congestionamento
    const float W_RSSI = 0.8;
    const float W_CONGESTION = 0.2;

    // Calcula o peso final
    weight = (rssiWeight * W_RSSI + congestionWeight * W_CONGESTION);

    // Garante que o peso nunca seja zero
    return std::max(0.1f, weight);
}

void MeshNode::updateLEDs() {
    unsigned long now = millis();
    if (now - lastLedUpdate >= LED_BLINK_INTERVAL) {
        ledState = !ledState;
        if(nodeId == 0) {
            nodeId = mesh.getNodeId();
            pinMode(LED_PIN, OUTPUT);
            digitalWrite(LED_PIN, LOW);

            nodes[nodeId] = NodeInfo();
            nodes[nodeId].id = nodeId;
            nodes[nodeId].active = true;
        }

        if (!nodes[nodeId].active) {
            digitalWrite(LED_PIN, LOW);
            return;
        }

        if (nodes[nodeId].onPath) {
                digitalWrite(LED_PIN, ledState);
        } else {
            digitalWrite(LED_PIN, HIGH);
        }
        lastLedUpdate = now;
    }
}

String MeshNode::getTopology() {
    JsonDocument doc;
    doc["type"] = "topology";

    JsonArray nodesArray = doc["nodes"].to<JsonArray>();
    for (const auto& node : nodes) {
        JsonObject nodeObj = nodesArray.add<JsonVariant>().to<JsonObject>();
        nodeObj["id"] = node.second.id;
        nodeObj["active"] = node.second.active;
        nodeObj["onPath"] = node.second.onPath;
        nodeObj["rssiThreshold"] = node.second.rssiThreshold;
    }

    JsonArray connsArray = doc["connections"].to<JsonArray>();
    for (const auto& node : nodes) {
        for (const auto& conn : node.second.connections) {
            if (node.first < conn.first) {
                JsonObject connObj = connsArray.add<JsonVariant>().to<JsonObject>();
                connObj["from"] = node.first;
                connObj["to"] = conn.first;
                connObj["rssi"] = conn.second;
                connObj["excluded"] = excludedPaths.count(std::minmax(node.first, conn.first)) > 0;
            }
        }
    }

    String result;
    serializeJson(doc, result);
    return result;
}

void MeshNode::broadcastNodeState() {
    JsonDocument doc;
    doc["type"] = static_cast<int>(CommandType::NODE_STATE);
    doc["id"] = nodeId;
    doc["active"] = nodes[nodeId].active;
    doc["rssiThreshold"] = nodes[nodeId].rssiThreshold;

    JsonObject rssiValues = doc["rssiValues"].to<JsonObject>();
    for (const auto& conn : nodes[nodeId].connections) {
        rssiValues[String(conn.first)] = conn.second;
    }

    String msg;
    serializeJson(doc, msg);
    mesh.sendBroadcast(msg);
}

void MeshNode::broadcastTopology() {
    JsonDocument doc;
    doc["type"] = static_cast<int>(CommandType::TOPOLOGY_UPDATE);
    doc["senderId"] = nodeId;

    JsonObject connections = doc["connections"].to<JsonObject>();
    auto connectedNodes = mesh.getNodeList();
    for (auto connId : connectedNodes) {
        if (nodes[nodeId].connections.count(connId)) {
            connections[String(connId)] = nodes[nodeId].connections[connId];
        }
    }

    String msg;
    serializeJson(doc, msg);
    mesh.sendBroadcast(msg);
}