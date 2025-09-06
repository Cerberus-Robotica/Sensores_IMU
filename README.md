
# Sensores_IMU

=======
TM32F411 IMU Sensor Fusion

Este projeto demonstra a integração de sensores IMU (MPU6050) e magnetômetro (QMC5883L) em um STM32F411, utilizando o algoritmo de fusão de sensores **Madgwick AHRS** para calcular a orientação do dispositivo.

O código lê os dados de acelerômetro, giroscópio e magnetômetro, processa usando o filtro de Madgwick e envia o ângulo de direção (`heading`) via UART.

---

