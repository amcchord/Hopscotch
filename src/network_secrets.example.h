#pragma once
// Copy to network_secrets.h (gitignored) before building your device.
#define HOPSCOTCH_WIFI_SSID "your-2.4GHz-network"
#define HOPSCOTCH_WIFI_PASSWORD "your-wifi-password"
// Generate with: python3 -c 'import secrets; print(secrets.token_hex(24))'
#define HOPSCOTCH_API_TOKEN "replace-with-a-random-device-token"
#define HOPSCOTCH_AP_PASSWORD "replace-with-a-unique-recovery-password"
