# Changelog

## [0.1.0] - 2025-11-10
### Added
- Pi5 deployment files (config/systemd/udev)
- CLI-based Python tools (driver monitor, lane detector, fingerprint logger)
- Flask dashboard scaffold for telemetry
- GitHub Actions CI + pre-commit (ruff/black)
- Optional face-rec requirements

### Changed
- Standardized serial protocol (CMD/ACK/STATE/DIST/LANE/WARN/ERR)
