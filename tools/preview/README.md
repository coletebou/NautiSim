# Headless preview / screenshots

Renders the app's real scene code (`apps/mobile/src/render`) in a browser with three.js's WebGL2
backend after running the real physics, so README screenshots can be made without a phone.

```sh
npx vite build --config tools/preview/vite.config.ts
(cd tools/preview/dist && python3 -m http.server 8765 &)
chromium --headless=new --use-angle=swiftshader --enable-unsafe-swiftshader --window-size=1280,720 \
  --virtual-time-budget=60000 --screenshot=docs/img/screenshot-rib.png \
  "http://localhost:8765/index.html?hull=rib&camera=chase&sea=slight&t=30&helm=0.3"
```

Query parameters: `hull` (rib, runabout, kvlcc2), `camera` (chase, bridge, orbit), `sea` (calm … veryRough),
`t` (simulated seconds), `throttle`, `helm`, `v0` (initial speed, m/s), `dist` (orbit distance factor).
