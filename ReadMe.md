# Moonleashed

Flipper Zero firmware based on [Unleashed](https://github.com/DarkFlippers/unleashed-firmware).

> [!WARNING]
> Experimental firmware for experimental use. Do not use it for anything illegal.
> This project is independent. It is not affiliated with Flipper Devices, the Unleashed team, or Moon Firmware.

## Status

Moonleashed is being rebuilt on Unleashed `unlshd-093` as a series of small pull requests that can each be reviewed on their own. The plan and its progress live in the [v2 epic](../../issues/19). Until those pull requests land, this branch is Unleashed with the changes listed below.

The previous line of development (v1) is kept on the [`v1/main`](../../tree/v1/main) branch and the `moonleashed-v1-final` tag; its latest work is on [`v1/nimble-coc-central`](../../tree/v1/nimble-coc-central).

## What is different from Unleashed

- **Moonleashed branding.** The firmware reports itself as Moonleashed, with no Unleashed update slideshow or About screens.
- **Sub-GHz runs from the SD card.** The app is `/ext/apps/Sub-GHz/subghz.fap`; the protocol library stays in firmware for Sub-GHz Remote, the CLI and other apps.
- **Level-up animations on the SD card.** They ship in the SD dolphin pack instead of firmware.
- **Bluetooth runs on the NimBLE host.** Update packages install ST's BLE HCILayer radio (link layer only), and NimBLE on the main core is the BLE host: the companion app, pairing with a PIN, battery and device information. Bonds made on stock firmware are not carried over; pair once.
- **BLE apps work on NimBLE.** BLE Remote and Bad USB over BLE run unmodified. The Flipper advertises as a keyboard only while such an app runs.
- **Apps can add their own GATT services** through the stock `ble_gatt_*` API; they go live on NimBLE when the app's profile starts.
- **LE Secure Connections**, with the elliptic-curve work on the chip's PKA accelerator (`bt pka_test` checks it).
- **Radio tests work on the HCILayer radio**: the `bt` CLI carrier, packet and RSSI tests (debug mode).
- **The extra beacon works**, so BLE Spam–style apps transmit again. While a beacon runs, the companion advertisement pauses; connections stay up.
- **Apps can connect as central** (`furi_ble_session`) while the companion keeps its link, and drop any link (`ble_link_disconnect`).
- **Apps can use GATT as a client** (`ble_gatt_client_*`, Moon-compatible): discover, read, write and subscribe on a central link.
- **Apps can open L2CAP channels** (`ble_l2cap_coc_*`, Moon-compatible) as client or server, with SDUs up to 2048 bytes.
- **Apps can advertise their own payload** (`furi_ble_adv_*`), once or continuously, beside the companion's link, and **relay raw L2CAP frames on a fixed CID** (`ble_l2cap_fixed_*`).
- **Apps can drive pairing** (`ble_security_*`): IO capability, passkey, numeric comparison and Secure Connections OOB. The companion's PIN pairing returns when the app ends.
- **Apps can serve GATT without a profile** (`ble_gatt_server_*`): define services at runtime, commit, and get writes and subscriptions.
- **The BLE host is lighter on RAM.** Radio frames go from the interrupt straight into NimBLE's buffers, with no queue or reader thread between; about 10 KB of heap comes back to apps.

Everything else follows upstream Unleashed. See its README and changelog for features.

## Build

```sh
./fbt COMPACT=1 DEBUG=0 updater_package
```

See [HowToBuild](/documentation/HowToBuild.md) for the toolchain.

## Credits

- [Unleashed Firmware](https://github.com/DarkFlippers/unleashed-firmware) by @xMasterX and the Unleashed team and contributors: the base of this firmware.
- [Flipper Devices](https://github.com/flipperdevices/flipperzero-firmware): the official firmware underneath it.

## License

GPLv3, as upstream. See [LICENSE](/LICENSE).
