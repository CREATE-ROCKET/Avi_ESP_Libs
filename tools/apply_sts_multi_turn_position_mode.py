from pathlib import Path


def replace_once(path: Path, old: str, new: str, label: str) -> None:
    text = path.read_text()
    if new in text:
        return
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: target count={count}, expected=1")
    path.write_text(text.replace(old, new, 1))


header = Path("include/STS3215.h")
replace_once(
    header,
    """  [[nodiscard]] esp_err_t configurePositionMode(Persistence persistence);\n  [[nodiscard]] esp_err_t configureStepMode(Persistence persistence);\n""",
    """  [[nodiscard]] esp_err_t configurePositionMode(Persistence persistence);\n  [[nodiscard]] esp_err_t configureMultiTurnPositionMode(Persistence persistence);\n  [[nodiscard]] esp_err_t configureStepMode(Persistence persistence);\n""",
    "STS3215.h method declaration",
)

source = Path("src/actuator/STS3215.cpp")
anchor = """esp_err_t STS3215::configureStepMode(Persistence persistence) {\n"""
implementation = """esp_err_t STS3215::configureMultiTurnPositionMode(\n    Persistence persistence) {\n  if (!initialized_ || !configuration_valid_)\n    return ESP_ERR_INVALID_STATE;\n  if (!validPersistence(persistence))\n    return ESP_ERR_INVALID_ARG;\n\n  // STS3215のmulti-turn absolute position controlではangle limitを\n  // min=0/max=0にし、Mode 0とPhase BIT4を組み合わせる。\n  // 単回転position modeの0..4095 limitとは別設定なので混同しない。\n  const uint8_t limits[]{0, 0, 0, 0};\n  esp_err_t operation =\n      writeEpRom(kMinimumPosition, limits, sizeof(limits), persistence);\n  const uint8_t mode = static_cast<uint8_t>(OperatingMode::position);\n  if (operation == ESP_OK)\n    operation = writeEpRom(kOperatingMode, &mode, 1, persistence);\n\n  const esp_err_t refresh = refreshConfiguration();\n  if (refresh != ESP_OK)\n    return refresh;\n  if (operation != ESP_OK)\n    return operation;\n  if (minimum_position_ != 0 || maximum_position_ != 0 ||\n      operating_mode_ != OperatingMode::position)\n    return ESP_ERR_INVALID_RESPONSE;\n\n  const esp_err_t feedback =\n      setFeedbackMode(FeedbackMode::multi_turn, persistence);\n  if (feedback != ESP_OK)\n    return feedback;\n  return (phase_ & kFeedbackBit) != 0U ? ESP_OK : ESP_ERR_INVALID_RESPONSE;\n}\n\n"""
text = source.read_text()
if implementation not in text:
    if text.count(anchor) != 1:
        raise SystemExit("STS3215.cpp configureStepMode anchor not found exactly once")
    source.write_text(text.replace(anchor, implementation + anchor, 1))

print("STS3215 multi-turn position mode patch applied")
