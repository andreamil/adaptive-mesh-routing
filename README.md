# Sistema de Roteamento Adaptativo em Rede Mesh com ESP32 e ESP8266

Este repositório contém o código-fonte e a documentação referente ao projeto de um sistema de roteamento adaptativo em uma rede mesh Wi-Fi formada por dispositivos ESP32 e ESP8266. O objetivo principal do projeto é implementar um algoritmo de roteamento dinâmico capaz de escolher rotas ideais entre nós da rede, levando em consideração métricas como qualidade de sinal (RSSI), congestionamento e estado (ativo/inativo) dos nós.

## Principais Características

- **Roteamento Adaptativo com Dijkstra Modificado:**  
  Utiliza o algoritmo de Dijkstra, adaptado para considerar múltiplos fatores (RSSI, congestionamento, estado do nó) no cálculo das rotas, garantindo um roteamento mais eficiente e robusto.
  
- **Formação Automática da Rede Mesh (PainlessMesh):**  
  A rede é formada automaticamente, sem a necessidade de um ponto de acesso central. Os dispositivos ESP32/ESP8266 se conectam entre si, criando uma topologia dinâmica e resiliente a falhas.
  
- **Visualização do Estado da Rede por LEDs:**  
  Cada nó indica, por meio de LEDs, se está ativo, se participa da rota atual ou se está inativo, facilitando a análise e entendimento do comportamento da rede.
  
- **Interface de Controle via Serial:**  
  Parâmetros da rede, como threshold de RSSI, exclusão manual de rotas e solicitação de rotas entre nós específicos, podem ser configurados e acompanhados em tempo real via interface serial e mensagens em formato JSON.
