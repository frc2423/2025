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

### Development issues

Error when creating a custom build (adding an apk using sidequest):

> A task failed. Check the tasks screen for more info. questnav-custom1.apk: /data/local/tmp/_stream.apk could not be installed [INSTALL_FAILED_UPDATE_INCOMPATIBLE: Package com.DerpyCatAviationLLC.QuestNav
signatures do not match previously installed version; ignoring!]

This can be resolved by uninstalling the current version of the app:

```bash
# Uninstall the app
adb uninstall com.DerpyCatAviationLLC.QuestNav
```
