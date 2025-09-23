# bl0942

Измененный компонент для ESPHOME, совместимый с LibreTiny. Добавлено ожидание получения полного пакета данных. Оптимизировано время публикации показаний.

Modified component for ESPHOME, compatible with LibreTiny. Added waiting for the full data package to be received. Optimized the time of publication of sensors.

```yaml
uart:
  - id: uart_bus1
    tx_pin: GPIO17
    rx_pin: GPIO16
    baud_rate: 4800
    stop_bits: 1
    data_bits: 8
  - id: uart_bus2
    #tx_pin: GPIO5
    rx_pin: GPIO18
    baud_rate: 4800
    stop_bits: 1
    data_bits: 8
  - id: uart_bus3
    #tx_pin: GPIO5
    rx_pin: RX
    baud_rate: 4800
    stop_bits: 1
    data_bits: 8

sensor:
  - platform: bl0942
    update_interval: 5s
    uart_id: uart_bus1
    id: channel_1
    voltage:
      name: ${upper_devicename} Voltage1
    current:
      name: ${upper_devicename} Current1
    power:
      name: ${upper_devicename} Power1
    energy:
      name: ${upper_devicename} Energy1
    frequency:
      name: ${upper_devicename} Power Frequency1
  - platform: bl0942
    update_interval: 5s
    uart_id: uart_bus2
    id: channel_2
    voltage:
      name: ${upper_devicename} Voltage2
    current:
      name: ${upper_devicename} Current2
    power:
      name: ${upper_devicename} Power2
    energy:
      name: ${upper_devicename} Energy2
    frequency:
      name: ${upper_devicename} Power Frequency2
  - platform: bl0942
    update_interval: 5s
    uart_id: uart_bus3
    id: channel_3
    voltage:
      name: ${upper_devicename} Voltage3
    current:
      name: ${upper_devicename} Current3
    power:
      name: ${upper_devicename} Power3
    energy:
      name: ${upper_devicename} Energy3
    frequency:
      name: ${upper_devicename} Power Frequency3
```
