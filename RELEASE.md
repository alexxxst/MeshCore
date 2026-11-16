# Releasing Firmware

GitHub Actions is set up to automatically build and release firmware.

It will automatically build firmware when one of the following tag formats are pushed.

- `companion-v1.0.0`
- `repeater-v1.0.0`
- `room-server-v1.0.0`

> NOTE: replace `v1.0.0` with the version you want to release as.

- You can push one, or more tags on the same commit, and they will all build separately.
- Once the firmware has been built, a new (draft) GitHub Release will be created.
- You will need to update the release notes, and publish it.

## Local builds

For local firmware builds, use `build.sh`. See [Building Firmware](./docs/building.md).
Local archives are named `<env>-<archive-version>.zip`, where the archive
version is `FIRMWARE_VERSION` with `FIRMWARE_VERSION_STRIP` removed.
