import FlightSection from '@/components/flight/FlightSection';
import Sections, { Nav } from '@/components/Sections';
import SmoothScroll from '@/components/SmoothScroll';

export default function Home() {
  return (
    <main>
      <SmoothScroll />
      <Nav />
      <FlightSection />
      <Sections />
    </main>
  );
}
