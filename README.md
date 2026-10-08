# Electric Fan-Jet Lab

What if a plane had jet engines with no fuel in them?

Just a battery and fans. A big intake fan, two more fans behind it, and a small
nozzle to squeeze the air into a jet. Six of those engines, three under each wing
of a small plane for four people.

This is a browser lab for that idea. Change the engines, the battery or the trip
and it tells you whether the plane gets off the ground, how far it goes and where
every watt of battery power ends up. Then fly it, from brake release to a full
stop on the next runway, with the air through the wing and the engines drawn live.

**Try it:** [Electric Fan-Jet Lab](https://claude.ai/artifact/3ghENPvWB2wYi1TkELYW5s)

## What the model says about the idea

The default design is the idea as I first sketched it.

| | |
| :-- | :-- |
| Takeoff run | 259 m |
| Range, landing with 20% battery | 137 km, 39 min in the air |
| Cruise | 220 km/h at 1,500 m, 133 kW from the battery |
| Weight | 1,660 kg, of which 450 kg is battery (113 kWh) |
| Battery power that ends up pushing the plane | 58% |

It flies. Then the lab started pushing back:

- **Take the stators out and it never leaves the ground.** Without fixed vanes
  between the fans, each fan adds its spin on top of the last one's. The blades
  stall and the air leaves spinning instead of going backwards.
- **The small nozzle costs range.** It makes a 345 km/h jet for a 220 km/h plane,
  and the difference is energy left behind in the wake. Opening the nozzle to 70%
  of the fan gives 141 km.
- **One fan beats three.** One fan per engine with that bigger nozzle flies about
  158 km. The extra fans, and the longer nacelles that hold them, cost more in
  weight and drag than they give back.

## How it works

Everything is first-order physics in plain JavaScript, SI units throughout.

**Engine** ([src/physics.js](src/physics.js)). Each fan is a mean-line stage with
its own motor and an equal share of the power. The Euler turbomachine equation
gives the work each fan adds and the swirl it leaves behind. Swirl carries on to
the next fan unless stators turn it back into pressure, or the next fan spins the
other way. The de Haller ratio flags blades that are close to stalling. The solver
finds the mass flow where the nozzle passes exactly the air the fans push, then
splits the shaft power into push, wake, swirl, blade, duct and nozzle losses.
Those parts add back up to the shaft power to machine precision, and there is a
test for it.

**Aircraft and mission.** Mass from the parts (airframe, motors at 4 kW/kg, fans,
nacelles, battery, people), a lift and drag polar with ground effect and intake
spillage, the standard atmosphere. The quick estimate integrates the takeoff run,
climbs in 100 m bands at the best rate of climb, cruises, descends at idle and
flies a 3 degree approach. `FlightSim` flies the same mission step by step with
simple speed, height and autothrottle loops, which is what the Fly tab shows.

**Air streaks** ([src/flow.js](src/flow.js)). The wing section is a Joukowski
airfoil, so the flow round it is exact 2D potential flow, scaled to the local
lift and joined to two Biot-Savart tip vortices. The fuselage is a slender-body
source line. The intakes and nozzles are rings of sources and sinks. Inside the
duct, continuity sets the speed and each fan sets the spin. Up to six thousand
particles ride that field in a Web Worker and are drawn as fading instanced
trails in [Three.js](https://threejs.org).

## Run it

```bash
npm start         # http://localhost:5173
npm test          # physics tests, Node's built-in runner
npm run build     # dist/fanjet-lab.html, one self-contained page
```

No install step. Node 20 or newer, and Three.js loads from a CDN.

## Files

| | |
| :-- | :-- |
| `src/physics.js` | atmosphere, fan stages, engine solver, aircraft, mission estimate, `FlightSim` |
| `src/layout.js` | where every part of the plane sits, shared by the 3D model and the flow |
| `src/flow.js` | the velocity field and the particles |
| `src/particles.worker.js` | runs the particles off the main thread |
| `src/scene.js` | the plane, the airfield, the wind-tunnel backdrop and the trail renderer |
| `src/app.js` | the panels, the engine diagram, the energy bar and the flight loop |
| `tools/bundle.mjs` | turns the modules into one HTML file |

## Limits

First-order estimates for exploring an idea, not engineering numbers. The air is
treated as incompressible, which is fair with blade tips held under 240 m/s. Motor,
battery and airframe figures are typical 2026 values.

## License

MIT
