import { Big_Shoulders_Display, IBM_Plex_Mono, IBM_Plex_Sans } from 'next/font/google';
import './globals.css';

const display = Big_Shoulders_Display({ subsets: ['latin'], weight: ['700', '900'], variable: '--font-display' });
const sans = IBM_Plex_Sans({ subsets: ['latin'], weight: ['400', '500', '600'], variable: '--font-sans' });
const mono = IBM_Plex_Mono({ subsets: ['latin'], weight: ['400', '500'], variable: '--font-mono' });

export const metadata = {
  title: 'Vyom — launch and landing simulator',
  description: 'Fly a reusable rocket to orbit and land its booster back on the pad. 6-DOF physics, G-FOLD landing guidance and Monte Carlo campaigns, checked against real Falcon 9 flights.',
  openGraph: { title: 'Vyom', description: 'Launch it. Bring it back. A 6-DOF reusable rocket simulator.', type: 'website' },
};

export const viewport = { themeColor: '#04060D', width: 'device-width', initialScale: 1, viewportFit: 'cover' };

export default function RootLayout({ children }) {
  return (
    <html lang="en" className={`${display.variable} ${sans.variable} ${mono.variable}`}>
      <body>{children}</body>
    </html>
  );
}
