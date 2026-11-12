// Texts of the viewer that the bench output does not carry. Everything here is optional: a
// scenario, variant, engine or quantity that is missing falls back to what the data says.

// Per scenario (docs/scenarios.md §4): title, one sentence of setup, one of what it tests and
// why it is hard. The theory statement and its class come from the data and from scenarios.js.
export const DESCRIPTIONS = {
  block: {
    title: 'Block on an incline',
    setup: 'A block slides down a 30° incline with friction.',
    tests: 'The basic check of sliding friction: the acceleration must be g(sin θ − μ cos θ) while a large flat contact stays flat, without drifting sideways or turning.',
  },
  disk: {
    title: 'Spinning disk',
    setup: 'A flat disk spins on a plane until friction stops it.',
    tests: 'Spin friction over an area: the stopping time depends on how the pressure is spread under the disk, which a single contact point cannot represent.',
  },
  cylinder: {
    title: 'Cylinder on an incline',
    setup: 'A cylinder lying on its side rolls down a 15° incline from rest.',
    tests: 'Rolling without slipping on a line contact: stiction must hold the contact while the body accelerates at (2/3) g sin θ, and the facets of the mesh must not brake it.',
  },
  push: {
    title: 'Pushed block',
    setup: 'A box at rest is pushed by a horizontal force that grows by 2 N every second.',
    tests: 'The passage from sticking to sliding: the box must stay still, without creeping, until the force reaches μmg, and then accelerate as the theory says.',
  },
  tip: {
    title: 'Tipping box',
    setup: 'A tall box stands on a plane while gravity is slowly tilted.',
    tests: 'The pressure under an eccentric load: on an elastic foundation the pressure shifts to the downhill edge and the box topples earlier than a rigid one, at a known angle.',
  },
  drop: {
    title: 'Dropped block',
    setup: 'A 10 kg box falls flat from 20 mm onto a plane and bounces.',
    tests: 'The normal force in an impact: stiffness and damping together set how deep the box sinks and how fast it rebounds.',
  },
  billiard: {
    title: 'Billiard ball',
    setup: 'A sphere is launched along a plane, sliding without spin.',
    tests: 'The passage from sliding to rolling: friction must spin the ball up and vanish when v = ωr, at a known time and speed, on a contact patch of a few millimetres.',
  },
  contensou: {
    title: 'Disk with spin and slip',
    setup: 'A disk slides and spins on a plane at the same time.',
    tests: 'The coupling of sliding and spinning friction (Contensou): both motions must stop together, which needs the friction integrated over the contact area.',
  },
  vgroove: {
    title: 'Block in a V-groove',
    setup: 'A square bar slides down a 90° V-groove inclined 30°.',
    tests: 'Two flat contacts at once on a ground that is not convex: the wedging multiplies the friction by √2 and the bar must stay aligned in the groove.',
  },
  ring: {
    title: 'Spinning ring',
    setup: 'A thick ring spins on a plane until friction stops it.',
    tests: 'Spin friction on a contact with a hole in it: the centroid of the contact lies where nothing touches, so a force applied there cannot stand for the whole area.',
  },
  hole: {
    title: 'Cube wedged in a hole',
    setup: 'A cube sinks under its own weight into a square hole 0.2 mm narrower than itself.',
    tests: 'An interference fit: four walls squeeze the cube, their normal forces cancel and only their friction holds it, so it must stop at a known depth and stay there.',
  },
  bowl: {
    title: 'Sphere in a bowl',
    setup: 'A sphere is released up the side of a spherical bowl and rolls back and forth.',
    tests: 'Rolling on a concave ground made of many flat pieces: the period and the amplitude must hold while the contact crosses the facets and the seams between pieces.',
  },
  trough: {
    title: 'Cylinder in a trough',
    setup: 'A cylinder is released up the side of a cylindrical trough and rolls back and forth.',
    tests: 'Rolling on a faceted concave ground with a line contact: the period must be that of the rolling pendulum and the swing must not die out.',
  },
  bearing: {
    title: 'Shaft in a bearing',
    setup: 'A shaft spins inside a hole of its own radius until friction stops it.',
    tests: 'A conforming curved contact: the friction torque depends on the pressure around the shaft, and the seams between the convex pieces of the bearing lie inside the contact.',
  },
  corner: {
    title: 'Cube in a corner',
    setup: 'A cube rests against the floor and the two walls of a corner, with gravity pointing into it.',
    tests: 'Three contacts at once, at rest: each face must sink by its share of the weight over K·a² and the cube must settle without moving on.',
  },
  offset: {
    title: 'Block with offset mass',
    setup: 'A flat slab whose centre of mass is off its centre rests on a plane.',
    tests: 'The moment a contact patch carries: the uneven pressure must tilt the slab by a small known angle and leave it at rest.',
  },
  twoblocks: {
    title: 'Two stacked blocks',
    setup: 'A block rides on another down a 30° incline; the lower one slides on the ground.',
    tests: 'Stiction between two moving bodies: the upper block must be held without any slip while the pair accelerates together.',
  },
  stack: {
    title: 'Stack of three cubes',
    setup: 'Three cubes are stacked on a plane and gravity is tilted to 5°.',
    tests: 'Stacking at rest: every interface carries the cubes above it with a known compression, and nothing may creep down the slope during the run.',
  },
  cone: {
    title: 'Spinning frustum',
    setup: 'A truncated cone spins in a matching conical seat until friction stops it.',
    tests: 'A conforming conical contact: the wedging in the seat multiplies the friction torque by 1/sin α, and the seams between the pieces of the seat lie inside the contact.',
  },
  sphere: {
    title: 'Sphere on an incline',
    setup: 'A sphere rolls down a 20° incline from rest.',
    tests: 'Rolling without slipping on a very small contact patch: stiction must hold while the sphere accelerates at (5/7) g sin θ.',
  },
  journal: {
    title: 'Shaft in a hole with clearance',
    setup: 'A shaft spins inside a wider hole, riding on its wall, until friction stops it.',
    tests: 'The line-contact twin of the bearing: here the friction torque is the same for any contact model, so it tells the errors of a method from the assumptions about a conforming contact.',
  },
  pieces1: {
    title: 'Ground in one piece (control)',
    setup: 'Two boxes slide to a stop on a table, one of them spinning, and the same two on a copy of the table 2 m away.',
    tests: 'The control of the cut grounds: both tables are the same single piece, so any difference between the two pairs of boxes is the repeatability of the method.',
  },
  pieces2: {
    title: 'Ground cut into 2 pieces',
    setup: 'Two boxes slide to a stop on a whole table, one of them spinning, and the same two on a table cut into 2 pieces.',
    tests: 'Independence from the decomposition: the boxes cross the seam between pieces and must move exactly as on the whole table.',
  },
  pieces8: {
    title: 'Ground cut into 8 pieces',
    setup: 'Two boxes slide to a stop on a whole table, one of them spinning, and the same two on a table cut into 8 pieces.',
    tests: 'Independence from the decomposition: the boxes cross several seams between pieces and must move exactly as on the whole table.',
  },
  pieces64: {
    title: 'Ground cut into 64 pieces',
    setup: 'Two boxes slide to a stop on a whole table, one of them spinning, and the same two on a table cut into 64 pieces.',
    tests: 'Independence from the decomposition: the boxes always rest on several pieces at once and must move exactly as on the whole table.',
  },
  wedge: {
    title: 'Blocks either side of the friction limit',
    setup: 'Two blocks sit on a plane that is tilted to 20°; one has μ just above tan 20°, the other just below.',
    tests: 'The static friction limit, 10 % either side of it: one block must hold without creeping and the other must slide at the rate of the theory.',
  },
  restitution: {
    title: 'Dropped spheres',
    setup: 'Four spheres fall onto a plane and reach it at 0.5, 1, 2 and 4 m/s.',
    tests: 'The damping of the normal force over a range of impact speeds: the law fixes each rebound speed, and no impact may create energy.',
  },
  oblique: {
    title: 'Oblique impacts',
    setup: 'Two spheres without spin hit a plane at an angle, one shallow and one steep.',
    tests: 'Friction during an impact that lasts a few steps: one ball must slide throughout and the other must end rolling, each with the spin the friction impulse gives it.',
  },
  edge: {
    title: 'Box sliding off a table',
    setup: 'A box slides without friction towards the edge of a table and falls off it.',
    tests: 'A contact that shrinks to a line: once the centre of mass passes the edge the box pitches about it, and a soft table lets it lean too early.',
  },
  turntable: {
    title: 'Ball on a turntable',
    setup: 'A ball rolls on a heavy disk that spins freely on a plane.',
    tests: 'Rolling on a moving body: the ball must circle at a fixed fraction of the table\'s rate and keep its radius, which needs stiction to hold for seconds.',
  },
  energy: {
    title: 'Frictionless sphere in a bowl',
    setup: 'A sphere slides back and forth in a bowl with no friction and no damping.',
    tests: 'Conservation of energy: nothing in the model dissipates, so any energy that appears or disappears comes from the time stepping as the contact crosses the facets.',
  },
  tower: {
    title: 'Tower of ten blocks',
    setup: 'Ten blocks are stacked on a plane that is tilted to 5° and held for 10 s.',
    tests: 'Many contacts at rest: with ten interfaces any creep or jitter adds up along the tower, and nothing may move.',
  },
  massratio: {
    title: '1000 kg on 1 kg',
    setup: 'A 1000 kg block rests on a 1 kg block on a plane.',
    tests: 'A mass ratio of 1000: the light block sits between two stiff contacts, which is where an explicit coupling of contact and engine becomes unstable.',
  },
};

// What the theory of a scenario rests on (the "class" field of a run).
export const THEORY_CLASSES = {
  A: {
    short: 'rigid-body result, the same for any contact model',
    long: 'Class A: a rigid-body result that does not depend on the contact model. Every model must converge to it as it stiffens, so only these scenarios rank contact models of different kinds against each other.',
  },
  B: {
    short: 'assumes a pressure distribution',
    long: 'Class B: the theory assumes a pressure distribution (uniform, Winkler…). Other admissible distributions give a band of answers, so a method inside the band is not wrong.',
  },
  C: {
    short: 'consistency with the Winkler law',
    long: 'Class C: consistency with the Winkler law. The theory contains the stiffness K or integrates the law\'s own force: it tells whether a method behaves as that foundation, not whether it is accurate.',
  },
};

// Known variants of a run. `detail` is used only while the data does not say the number of
// segments or the time step; a variant that is not listed shows its own name.
export const VARIANTS = {
  base: { label: 'Base', detail: '1 ms', note: 'The reference run: base meshes (64 segments per turn on curved surfaces) and the base time step of 1 ms.' },
  fine: { label: 'Fine mesh', detail: '', note: 'The same run with curved surfaces twice as fine. A result that changes here depends on the mesh resolution.' },
  dt: { label: 'Small step', detail: '0.2 ms', note: 'The same run with a time step five times smaller. A result that changes here depends on the step.' },
  perturbed: { label: 'Perturbed start', detail: '', note: 'The same run with every body starting 1 mm/s and 0.01 rad/s off. A result that changes here is sensitive to the initial conditions.' },
  mesh32: { label: 'Coarse mesh', detail: '', note: 'Curved surfaces with half the segments of the base run (mesh series: 32, 64, 128).' },
  mesh128: { label: 'Fine mesh', detail: '', note: 'Curved surfaces with twice the segments of the base run (mesh series: 32, 64, 128).' },
  dt2: { label: 'Step 2 ms', detail: '2 ms', note: 'Step series: the time step doubled with respect to the base run (2, 1, 0.5, 0.2, 0.1 ms).' },
  'dt0.5': { label: 'Step 0.5 ms', detail: '0.5 ms', note: 'Step series: half the base time step.' },
  'dt0.2': { label: 'Step 0.2 ms', detail: '0.2 ms', note: 'Step series: a fifth of the base time step.' },
  'dt0.1': { label: 'Step 0.1 ms', detail: '0.1 ms', note: 'Step series: a tenth of the base time step.' },
  stiff: { label: 'Ten times stiffer', detail: '0.2 ms', note: 'Foundation stiffness K times 10 at a step of 0.2 ms, towards the rigid limit (classes A and B only).' },
  ...Object.fromEntries([1, 2, 3, 4, 5, 6, 7, 8].map(k => [`perturbed${k}`, {
    label: `Perturbed start ${k}`, detail: '',
    note: 'The same run with every body starting 1 mm/s and 0.01 rad/s off, in a direction drawn with seed ' + k + '. The eight perturbations give an interval, not replicates of the error.',
  }])),
  default: { label: 'Single run', detail: '', note: 'A run made outside the battery, with the options given to the bench.' },
};

// Display names where capitalizing the identifier is not enough.
export const NAMES = { mujoco: 'MuJoCo' };

// Names of the plotted quantities, by the first word of the series name; the rest of the name
// follows in brackets (v_lower: "Velocity (lower)").
export const QUANTITIES = {
  v: 'Velocity', w: 'Angular velocity', wr: 'Rim speed ω·r', angle: 'Angle', theta: 'Tilt of gravity',
  rotation: 'Rotation', tilt: 'Tilt', depth: 'Depth', delta: 'Penetration', penetration: 'Penetration',
  compression: 'Compression', creep: 'Creep', slip: 'Relative slip', lateral: 'Lateral shift',
  pitch: 'Pitch', energy: 'Energy', travel: 'Travel', displacement: 'Displacement',
};

// What the battery recorded for a run that did not end in a plain error number.
export const STATUSES = {
  nan: { label: 'nan', why: 'The run finished but its error is not a number: the trajectory diverged or the metric is undefined.' },
  timeout: { label: 'timeout', why: 'The run exceeded the wall-clock limit of the battery and was stopped; there is no trajectory.' },
  failed: { label: 'failed', why: 'The engine stopped with an error before the end of the scenario; there is no trajectory.' },
};

// The variants shown in a first row of groups, each group a second row of members.
export const VARIANT_GROUPS = [
  { id: 'base', label: 'Base', test: v => v === 'base' },
  { id: 'mesh', label: 'Mesh', test: v => v === 'fine' || /^mesh\d+$/.test(v) },
  { id: 'dt', label: 'Time step', test: v => v === 'dt' || /^dt[\d.]+$/.test(v) },
  { id: 'stiff', label: 'Stiffness', test: v => v === 'stiff' },
  { id: 'perturbed', label: 'Perturbed start', test: v => /^perturbed\d*$/.test(v) },
];
