# 2025

## Quest Nav

### adb commands

```bash
# list available wifi commands
adb shell cmd wifi list-scan-results
```

```bash
# Connect to robot network
adb shell cmd wifi connect-network "FRC-2423-comp" wpa2 "kwarqs2423"
```

```bash
# Check currently connected network
adb shell "dumpsys wifi | grep SSID"
```

```bash
# Check current network IP address
adb shell ip addr show wlan0
```

```bash
# Check if wifi is enabled
adb shell "dumpsys wifi | grep 'Wi-Fi is'"
```

```bash
# Confirm communication with robot by pinging it
adb shell ping -c 4 10.24.23.2
```

