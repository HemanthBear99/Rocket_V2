// Every number here comes from the Vyom simulator or from published
// Falcon 9 webcast telemetry (shahar603/Telemetry-Data, public domain).

// Latest 100-run Monte Carlo campaign (seed 42): landing error (m, capped at
// 300), touchdown speed (m/s), mission success.
const MC_RAW =
  '7.4,4.33,1;2.0,4.49,1;1.1,4.44,1;0.1,4.38,1;0.2,4.43,1;0.2,4.30,1;0.4,4.44,1;0.1,4.52,1;1.9,4.31,1;25.7,4.49,1;12.8,4.78,1;0.2,4.39,1;0.3,4.42,1;0.3,4.34,1;0.1,4.51,1;0.9,4.29,1;2.1,4.61,1;0.0,4.40,1;0.7,4.33,1;0.1,4.47,1;50.3,4.78,0;0.4,4.27,1;1.3,4.45,1;0.3,4.21,1;0.1,4.48,1;0.1,4.44,1;0.7,4.41,1;0.2,4.28,1;0.9,4.41,1;0.0,4.53,1;1.3,4.30,1;0.3,4.65,1;1.3,4.54,1;69.0,4.29,0;2.7,4.96,1;0.3,4.34,1;0.5,4.42,1;0.3,4.49,1;1.4,4.54,1;0.2,4.43,1;2.9,4.64,1;300.0,4.59,0;0.9,4.36,1;0.1,4.46,1;0.4,4.37,1;2.7,4.65,1;0.2,4.54,1;0.3,4.34,1;0.2,4.48,1;0.4,4.39,1;0.5,4.26,1;4.4,4.55,1;0.3,4.27,1;0.2,4.36,1;0.8,4.22,1;0.2,4.29,1;0.5,4.44,1;267.6,18.11,0;0.1,4.53,1;0.8,4.30,1;0.6,4.46,1;0.1,4.50,1;12.7,4.63,1;3.7,4.34,1;0.3,4.59,1;32.7,4.84,1;0.5,4.39,1;0.3,4.34,1;300.0,4.55,0;0.8,4.45,1;1.2,4.41,1;0.6,4.30,1;0.3,4.40,1;0.5,4.50,1;0.3,4.47,1;4.5,4.66,1;0.5,4.35,1;1.6,4.50,1;0.4,4.31,1;0.3,4.48,1;0.4,4.40,1;0.1,4.47,1;2.8,4.40,1;1.2,4.86,1;3.7,4.62,1;1.1,4.48,1;29.4,4.59,1;0.0,4.37,1;0.7,4.40,1;3.4,4.58,1;0.0,4.69,1;0.6,4.41,1;0.3,4.37,1;0.1,4.49,1;0.1,4.38,1;2.0,4.60,1;2.8,4.26,1;47.2,4.71,1;0.1,4.43,1;57.3,4.54,0';

export const MONTE_CARLO = MC_RAW.split(';').map((r) => {
  const [error, speed, ok] = r.split(',').map(Number);
  return { error, speed, ok: ok === 1 };
});

// Altitude (km) every 2 s from liftoff to stage-1 cutoff.
export const CURVES = {
  'CRS-11': [0.0,0.0,0.0,0.0,0.1,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.9,1.1,1.3,1.5,1.7,2.0,2.3,2.6,2.9,3.3,3.6,4.0,4.5,4.9,5.4,5.9,6.4,6.9,7.4,8.0,8.6,9.1,9.8,10.4,10.9,11.7,12.4,13.2,13.9,14.8,15.6,16.5,17.4,18.4,19.4,20.4,21.5,22.6,23.8,25.0,26.2,27.6,28.9,30.3,31.7,33.2,34.7,36.3,37.9,39.5,41.3,43.0,44.9,46.7,48.6,50.5,52.6,54.6,56.8,59.0,61.1],
  'NROL-76': [0.0,0.0,0.0,0.1,0.1,0.2,0.2,0.3,0.4,0.6,0.7,0.9,1.4,1.6,1.8,2.0,2.2,2.5,2.8,3.1,3.4,3.8,4.2,4.6,5.0,5.4,5.9,6.4,7.0,7.5,8.1,8.7,9.4,10.1,10.8,11.6,12.4,13.2,14.0,14.8,15.8,16.8,17.8,18.8,19.9,21.1,22.2,23.4,24.7,26.0,27.4,28.8,30.3,31.8,33.4,35.0,36.7,38.5,40.3,42.2,44.2,46.2,48.3,50.5,52.7,55.0,57.3,59.9,62.3,64.9,67.8],
  Zuma: [0.0,0.0,0.0,0.0,0.1,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.9,1.1,1.3,1.5,1.7,2.0,2.3,2.6,2.9,3.3,3.6,4.0,4.5,4.9,5.3,5.8,6.3,6.8,7.3,7.8,8.3,8.9,9.6,10.2,10.9,11.6,12.3,13.1,13.9,14.8,15.7,16.6,17.5,18.5,19.6,20.7,21.8,23.0,24.2,25.4,26.7,28.0,29.4,30.9,32.3,33.9,35.4,37.0,38.7,40.4,42.2,44.0,45.9,47.8,49.8,51.8,54.0,56.1,58.3,60.6],
  'Orbcomm OG2': [0.0,0.0,0.0,0.0,0.1,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.9,1.0,1.2,1.4,1.7,1.9,2.2,2.5,2.8,3.1,3.5,3.8,4.2,4.7,5.1,5.6,6.2,6.7,7.2,7.9,8.5,9.2,9.8,10.6,11.3,12.1,12.9,13.8,14.7,15.6,16.6,17.7,18.7,19.8,20.9,22.1,23.4,24.6,26.0,27.4,28.8,30.3,31.9,33.5,35.1,37.0,38.8,40.6,42.6,44.6,46.8,49.0,51.2,53.6,56.1,58.8,61.4,64.0,67.0,69.9,73.0],
  Vyom: [0.0,0.0,0.0,0.0,0.1,0.2,0.2,0.4,0.5,0.6,0.8,1.0,1.2,1.4,1.7,2.0,2.3,2.6,3.0,3.4,3.8,4.2,4.7,5.2,5.8,6.4,7.0,7.6,8.2,8.9,9.5,10.2,10.9,11.6,12.4,13.1,13.9,14.6,15.4,16.3,17.2,18.1,19.1,20.2,21.3,22.4,23.7,24.9,26.3,27.6,29.1,30.6,32.2,33.8,35.5,37.3,39.1,41.0,43.0,45.1,47.2,49.4,51.8,54.1,56.6,59.2,61.9,64.6,67.1],
};

export const FLIGHTS = [
  { name: 'SpaceX CRS-11', real: '1,684', sim: '1,618', diff: '−3.9%', bbReal: '161 s', bbSim: '164 s' },
  { name: 'Orbcomm OG2', real: '1,670', sim: '1,692', diff: '+1.4%', bbReal: '—', bbSim: '—' },
  { name: 'NROL-76', real: '1,685', sim: '1,642', diff: '−2.5%', bbReal: '157 s', bbSim: '157 s' },
  { name: 'Zuma', real: '1,672', sim: '1,629', diff: '−2.6%', bbReal: '159 s', bbSim: '160 s' },
];

// Chapter captions for the scroll flight, keyed to scroll progress.
export const CHAPTERS = [
  { a: 0.035, b: 0.10, t: 'T−0 · LC-39A', title: 'Nine engines.\nOne way up.', text: '433 tonnes of first stage and 7.6 MN of thrust. Scroll to light them.' },
  { a: 0.10, b: 0.22, t: 'T+0 s · Liftoff', title: 'Clear of\nthe tower', text: 'Thrust-to-weight 1.40. The gravity turn starts ten seconds in.' },
  { a: 0.22, b: 0.35, t: 'T+63 s · Max-q', title: '31.9 kPa,\nheld', text: 'A physics-based throttle law holds dynamic pressure 3 kPa under the structural limit.' },
  { a: 0.35, b: 0.45, t: 'T+130 s · Separation', title: 'Two vehicles\nnow', text: 'Stage 2 lights for orbit at 1,650 m/s. The booster flips on cold-gas thrusters.' },
  { a: 0.45, b: 0.58, t: 'T+157 s · Boostback', title: 'Turn it\naround', text: 'Three engines cancel 1,400 m/s of downrange speed and aim it back at the coast.' },
  { a: 0.60, b: 0.75, t: 'T+341 s · Entry burn', title: 'Through\nthe fire', text: 'Slowing from 1,190 m/s before the thick air. Grid fins take over the steering.' },
  { a: 0.80, b: 0.93, t: 'T+456 s · Landing burn', title: 'One engine.\nNo second try.', text: 'Convex-optimization guidance plans the fuel-optimal path down to the pad.' },
  { a: 0.945, b: 1.2, t: 'T+505 s · Touchdown', title: '4.41 m/s.\n0.3 m.', text: 'Booster safe on LZ-1. Now run it a hundred times with everything dispersed.' },
];

export const SPECS = [
  { k: 'Vehicle builder', title: 'Design it, then check it', text: 'Engines, thrust, diameter, masses and payload. A feasibility planner checks delta-v, thrust-to-weight and recovery fuel before you launch.' },
  { k: 'Landing guidance', title: 'G-FOLD on a switch', text: 'Flip between a classic suicide burn and lossless-convexification G-FOLD, the fuel-optimal method from the planetary-landing literature.' },
  { k: 'Monte Carlo', title: 'Hundreds of missions', text: 'Randomise thrust, Isp, mass, wind, density and pitch-over together. Get success rates with confidence intervals and a failure breakdown.' },
  { k: 'Physics', title: 'Six degrees of freedom', text: 'RK4 integration, EGM96 gravity, US-76 atmosphere, Mach-dependent aerodynamics, grid fins, landing legs, slosh and actuator limits.' },
  { k: 'Your data', title: 'Bring tunnel or CFD results', text: 'Import drag, lift and moment tables as CSV. Vyom checks the grid, charts it against the built-in model and flies with it.' },
  { k: 'Reports', title: 'Compare, then export', text: 'Every run is saved. Overlay two missions or two campaigns, see exactly which settings differ, and export the report as a PDF.' },
];

export const PLANS = [
  { name: 'Student', who: 'Personal licence for one learner, on one PC.', inr: '₹2,999', usd: '$49', per: '/ year', items: ['Full simulator and 3D view', 'Monte Carlo up to 200 runs', 'Saved scenarios and run comparison', 'Community support'], cta: 'Start free trial', href: '#download' },
  { name: 'Academic Lab', who: 'For a university course or research group.', inr: '₹34,999', usd: '$699', per: '/ lab / year', hot: true, items: ['Up to 30 seats in one department', 'Unlimited Monte Carlo campaigns', 'G-FOLD guidance and aero data import', 'Teaching scenarios and lab exercises', 'Email support'], cta: 'Start free trial', href: '#download' },
  { name: 'Startup', who: 'For launch-vehicle teams doing early design trades.', inr: '₹1,99,999', usd: '$4,900', per: '/ year', items: ['5 named seats', 'Import your tunnel and CFD data', 'Validation report templates', 'Priority support, 2-day reply'], cta: 'Start free trial', href: '#download' },
  { name: 'Enterprise', who: 'Custom vehicles, on-site deployment and training.', inr: 'Custom', usd: 'Custom', per: '', items: ['Unlimited seats', 'Custom vehicle architectures', 'Source-level integration', 'Training for your team'], cta: 'Contact sales', href: 'mailto:sales@example.com' },
];

export const FAQS = [
  { q: 'Can I use Vyom for flight-critical decisions?', a: 'No. It is an engineering and teaching tool for early design studies, guidance research and coursework. Its aerodynamics come from geometry and published literature, not from wind-tunnel tests of your vehicle. The validation section lists where it matches flight data and where it does not.' },
  { q: 'Which vehicles can I model?', a: 'Two-stage vehicles with a reusable first stage returning to the launch site. You set engine count and thrust, masses, diameter and payload. Drone-ship landings, three-stage vehicles and solid motors are on the roadmap.' },
  { q: 'Does it need an internet connection?', a: 'No. Everything runs on your PC, and your scenarios and results stay in your own data folder. You only need the internet to download the installer.' },
  { q: 'How does the trial work?', a: 'Install it and you get 14 days with every feature. When you buy, you receive a licence key to paste into Settings. Your saved scenarios and runs carry over.' },
  { q: 'Is the flight at the top of this page real?', a: 'Its timeline, speeds, altitudes and touchdown numbers come from the default simulated mission. The 3D scene is stylised: altitudes and the curve of the Earth are exaggerated so the whole flight fits on screen. Earth imagery is NASA Blue Marble and Black Marble.' },
];

export const DOWNLOAD_URL = '#download'; // replace with the installer URL when it is hosted
