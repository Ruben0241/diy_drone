# Mac-App (Electron)

Der Configurator als eigene App, ohne Browser und ohne Terminal-Fenster.

## Bauen (macOS oder Linux)

```bash
cd tools/configurator
mkdir -p /tmp/cfgapp && cp index.html protocol.js mock.js mac/main.js mac/package.json /tmp/cfgapp/
npx @electron/packager /tmp/cfgapp "DIY Drone Configurator" --platform=darwin --arch=arm64 \
  --electron-version=45.4.1 --icon=mac/icon.icns --app-bundle-id=ch.kzu.diydrone.configurator --out=dist
```

## Installieren (einmalig, die App ist nicht von Apple signiert)

```bash
cd ~/Downloads && xattr -cr "DIY Drone Configurator.app" && codesign --force --deep --sign - "DIY Drone Configurator.app" && mv "DIY Drone Configurator.app" /Applications/
```

`main.js` wählt beim «Verbinden» den Flightcontroller (USB 1209:0001) automatisch,
speichert Downloads (CSV, Protokoll) direkt im Ordner Downloads und fragt beim Schliessen
nach, wenn noch etwas ungespeichert ist.
