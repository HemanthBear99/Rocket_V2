# Vyom website

Next.js 14 + React Three Fiber marketing site with a scroll-driven 3D flight.

```bash
npm install
npm run dev      # http://localhost:3000
npm run build    # production build
```

## Deploy on Vercel
1. Import the GitHub repo in Vercel.
2. Set **Root Directory** to `website` (framework preset: Next.js).
3. Deploy, then add your domain under Project → Settings → Domains.

Before launch, set the real installer link in `lib/data.js` (`DOWNLOAD_URL`)
and replace `sales@example.com` with your sales address.

## Where things live
- `components/flight/` — the WebGL flight: `FlightScene.jsx` (choreography,
  camera, bloom), `Rocket.jsx`, `Earth.jsx`, `LaunchSite.jsx`, `particles.js`.
- `lib/mission.js` — scroll progress → vehicle pose and HUD numbers, keyed to
  the default simulated mission.
- `lib/data.js` — every number shown on the page (Monte Carlo runs, Falcon 9
  curves, pricing, FAQ).

Earth imagery: NASA Blue Marble and Black Marble (public domain).
Flight data: shahar603/Telemetry-Data (public domain).
