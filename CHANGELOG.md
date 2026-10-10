# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- Host regression tests: the real firmware against Arduino/NimBLE test doubles, with
  sanitizers in CI (`make test`, `make sanitize`).
- `Makefile` with `check`, `deps`, `compile`, `upload` and `monitor`.
- `docs/how-it-works.md`, `docs/testing.md` with a hardware checklist, `AGENTS.md`,
  `CONTRIBUTING.md`.

### Changed
- ESP32 Arduino core 3.3.12 and NimBLE-Arduino 2.5.1.
- Start/Resume is forwarded to the trainer as well as answered locally.
- The command queue merges repeated updates only from the same client, so every app gets a
  response.

### Fixed
- Request Control is actually sent to the trainer on connection (it was skipped).
- Trainer responses must match the pending command's opcode.
- A trainer response arriving before the write returned was dropped, and the command then
  timed out.
- On trainer disconnect, pending and queued commands fail immediately instead of leaving
  apps waiting.
- Failure responses with no sender (serial console commands, or an app that left) were
  indicated to every client, including Garmin.
- Control Point transaction state is locked between the Bluetooth task and the main loop.
