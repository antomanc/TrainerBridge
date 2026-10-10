# Instructions for AI agents working on TrainerBridge

TrainerBridge is ESP32 Arduino firmware (C++17) that proxies one FTMS smart trainer to
several BLE clients. Read [docs/how-it-works.md](docs/how-it-works.md) first.

- Run `make sanitize` before finishing. `make compile` needs `arduino-cli` and `make deps`;
  if it isn't installed, CI runs it on the pull request.
- Pinned versions: ESP32 Arduino core 3.3.12, NimBLE-Arduino 2.5.1 (Makefile, CI, README).
  Bump them together.
- Never write to the trainer from a NimBLE callback: queue the command and let
  `processQueuedCommands()` in `loop()` send it.
- Control Point transaction state (`cpTxState`, `pendingCp`, `pendingCpSince`) is shared
  with the NimBLE host task. Touch it only inside `cpTxMux`, and make BLE/Serial calls after
  leaving the critical section.
- Control Point responses go only to the originating connection handle. Never indicate to
  all clients: Garmin drops sensors that send unexpected indications.
- Add a named test to `tests/firmware_test.cpp` for any behaviour you change. Extend the
  stubs in `tests/stubs/` only as far as the firmware uses the real API.
- Nothing here replaces a ride on real hardware: say so in the PR and point to the
  checklist in [docs/testing.md](docs/testing.md).
