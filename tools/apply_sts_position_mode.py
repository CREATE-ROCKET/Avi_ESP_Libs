from pathlib import Path

header = Path("include/STS3215.h")
text = header.read_text()
old = """  [[nodiscard]] esp_err_t setOperatingMode(OperatingMode mode,\n                                           Persistence persistence);\n  [[nodiscard]] esp_err_t configureStepMode(Persistence persistence);\n"""
new = """  [[nodiscard]] esp_err_t setOperatingMode(OperatingMode mode,\n                                           Persistence persistence);\n  [[nodiscard]] esp_err_t configurePositionMode(Persistence persistence);\n  [[nodiscard]] esp_err_t configureStepMode(Persistence persistence);\n"""
if new not in text:
    if text.count(old) != 1:
        raise SystemExit("STS3215.h target block not found exactly once")
    header.write_text(text.replace(old, new, 1))

source = Path("src/actuator/STS3215.cpp")
text = source.read_text()
anchor = """esp_err_t STS3215::configureStepMode(Persistence persistence) {\n"""
implementation = """esp_err_t STS3215::configurePositionMode(Persistence persistence) {\n  if (!initialized_ || !configuration_valid_)\n    return ESP_ERR_INVALID_STATE;\n  if (!validPersistence(persistence))\n    return ESP_ERR_INVALID_ARG;\n\n  // Step modeがminimum/maximum positionを0へ変更するため、\n  // position modeへ戻す際は360度の通常範囲も同時に復元する。\n  const uint8_t minimum[]{0x00, 0x00};\n  const uint8_t maximum[]{0xFF, 0x0F};\n  esp_err_t operation =\n      writeEpRom(kMinimumPosition, minimum, sizeof(minimum), persistence);\n  if (operation == ESP_OK)\n    operation =\n        writeEpRom(kMaximumPosition, maximum, sizeof(maximum), persistence);\n  const uint8_t mode = static_cast<uint8_t>(OperatingMode::position);\n  if (operation == ESP_OK)\n    operation = writeEpRom(kOperatingMode, &mode, 1, persistence);\n\n  const esp_err_t refresh = refreshConfiguration();\n  if (refresh != ESP_OK)\n    return refresh;\n  if (operation != ESP_OK)\n    return operation;\n  return minimum_position_ == 0 && maximum_position_ == 4095 &&\n                 operating_mode_ == OperatingMode::position\n             ? ESP_OK\n             : ESP_ERR_INVALID_RESPONSE;\n}\n\n"""
if implementation not in text:
    if text.count(anchor) != 1:
        raise SystemExit("STS3215.cpp configureStepMode anchor not found exactly once")
    source.write_text(text.replace(anchor, implementation + anchor, 1))

print("STS3215 position mode configuration patch applied")
