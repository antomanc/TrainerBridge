# Contributing

Issues and PRs welcome, especially reports from trainers other than the Van Rysel D100.

## Development

```bash
make test       # host tests, needs only g++
make check      # host tests with sanitizers + firmware build (needs arduino-cli, `make deps` once)
make upload     # build and flash, PORT=/dev/ttyUSB0 by default
make monitor    # serial console
```

The host tests run the real firmware code against Arduino and NimBLE test doubles, so most
logic can be tested without a board. See [docs/testing.md](docs/testing.md) for what they
cover and the hardware checklist.

## Guidelines

- Keep the firmware warning-free: CI builds with `--warnings all`.
- Bluetooth callbacks must not block. Writes to the trainer go through the command queue.
- Add or update a named test in `tests/firmware_test.cpp` for behaviour changes.
- Change user-facing settings in `Config.h` and the table in the README together.
- Mention in the PR whether you tested on hardware, with which trainer, app and Garmin.

## Reporting a trainer that doesn't work

Set `DEBUG_LOG 1` in `Config.h`, flash, and attach the serial log from boot until the
problem, plus the trainer model and the app or Garmin you used.
