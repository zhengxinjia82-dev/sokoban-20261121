// Sokoban solver: push-optimal per room, rooms solved independently and concatenated.
//
// The levels are a hallway with rooms off it; a box pushed out of a room's door can
// never come back, so every box stays in its own room and the level's push optimum is
// the sum of its rooms' optima. This solver finds each room (a connected floor region
// off the hallway), solves it push-optimally with A* over pushes (matching heuristic
// + simple-deadlock pruning), and walks the player between rooms through the hallway.
// Local run: mkdir -p out && MAKEFUN_INPUT_DIR=<kit>/practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Four moves: up, down, left, right. A push step is written upper-case.
const DIRS = [
  { dx: 0, dy: -1, ch: 'u' },
  { dx: 0, dy: 1, ch: 'd' },
  { dx: -1, dy: 0, ch: 'l' },
  { dx: 1, dy: 0, ch: 'r' },
];

interface Level {
  w: number;
  h: number;
  wall: Uint8Array; // 1 = wall, 0 = floor (room floor or hallway)
  goals: number[];
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = 0;
  rows.forEach((row, y) => {
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#';
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if ('.*+'.includes(ch)) goals.push(i);
      if ('$*'.includes(ch)) boxes.push(i);
      if ('@+'.includes(ch)) player = i;
    }
  });
  return { w, h, wall, goals, boxes, player };
}

/** Shortest player walk from `from` to `to` over floor cells, boxes as walls. Returns the
 *  move letters (lower-case), or null when unreachable. */
function walkMoves(level: Level, boxSet: Set<number>, from: number, to: number): string | null {
  if (from === to) return '';
  const { w, h, wall } = level;
  const prev = new Int32Array(w * h).fill(-1);
  const step = new Uint8Array(w * h); // direction index used to reach the cell
  const q = [from];
  prev[from] = from;
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi];
    if (c === to) break;
    const cx = c % w;
    const cy = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DIRS[d]!.dx;
      const ny = cy + DIRS[d]!.dy;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const j = ny * w + nx;
      if (wall[j] || boxSet.has(j) || prev[j] !== -1) continue;
      prev[j] = c;
      step[j] = d;
      q.push(j);
    }
  }
  if (prev[to] === -1) return null;
  let moves = '';
  for (let c = to; c !== from; c = prev[c]!) moves = DIRS[step[c]!]!.ch + moves;
  return moves;
}

interface Room {
  cells: number[]; // floor cells of the room (global indices)
  cellSet: Set<number>;
  boxes: number[];
  goals: number[];
  door: number; // room cell adjacent to the hallway
  entry: number; // hallway cell outside the door
}

/** Split a level into its rooms: the hallway is the player's row; every connected floor
 *  region off it is one room, opening onto the hallway through a single door cell. */
function findRooms(level: Level): Room[] {
  const { w, h, wall, goals, boxes, player } = level;
  const hallRow = (player / w) | 0;
  const goalSet = new Set(goals);
  const boxSet = new Set(boxes);
  const seen = new Uint8Array(w * h);
  const rooms: Room[] = [];
  for (let i = 0; i < w * h; i++) {
    if (wall[i] || seen[i] || (i / w | 0) === hallRow) continue;
    const comp: number[] = [];
    const q = [i];
    seen[i] = 1;
    for (let k = 0; k < q.length; k++) {
      const c = q[k]!;
      comp.push(c);
      const cx = c % w;
      const cy = (c / w) | 0;
      for (const d of DIRS) {
        const nx = cx + d.dx;
        const ny = cy + d.dy;
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
        const j = ny * w + nx;
        if (wall[j] || seen[j] || (j / w | 0) === hallRow) continue;
        seen[j] = 1;
        q.push(j);
      }
    }
    const cellSet = new Set(comp);
    const rBoxes = boxes.filter((b) => cellSet.has(b));
    const rGoals = goals.filter((g) => cellSet.has(g));
    const doors = comp.filter((c) => Math.abs(((c / w) | 0) - hallRow) === 1);
    if (doors.length !== 1) continue; // not a single-door room; skip it
    const door = doors[0]!;
    const entry = ((door / w) | 0) < hallRow ? door + w : door - w;
    rooms.push({ cells: comp, cellSet, boxes: rBoxes, goals: rGoals, door, entry });
  }
  return rooms;
}

// ---- Per-room push-optimal A* search ----
// A state is the sorted boxes plus the player's square. The search steps are pushes:
// from a state the player walks (BFS, boxes as walls) to a pushing square, then pushes
// one box one step. Cost is one per push. The heuristic is the minimum-weight matching
// between boxes and goals under shortest-path push distance (admissible: every box needs
// at least its shortest-path pushes to some distinct goal). Simple deadlocks (corners
// and goal-less wall runs) are pruned. The player's square is normalised to the lowest
// reachable square, so states that differ only in where the player stands collapse.

interface RoomSolve {
  moves: string;
  endPlayer: number; // global cell where the player ends
}

function solveRoom(level: Level, room: Room, boxes0: number[], startPlayer: number): RoomSolve | null {
  const { w } = level;
  const cells = [...room.cells];
  const n = cells.length;
  const ENTRY = n; // local index of the hallway square outside the door (player only)
  const local = new Map<number, number>();
  cells.forEach((c, i) => local.set(c, i));
  const goalSet = new Set(room.goals.map((g) => local.get(g)!));
  const goalList = [...goalSet];
  const nb = boxes0.length;

  // nbr[i][d]: local index of the neighbour in direction d, -1 for a wall, ENTRY for the door's hall square.
  const nbr: number[][] = [];
  for (let i = 0; i <= n; i++) {
    nbr.push([-1, -1, -1, -1]);
  }
  const globalOf = (i: number): number => (i === ENTRY ? room.entry : cells[i]!);
  for (let i = 0; i < n; i++) {
    const c = cells[i]!;
    const cx = c % w;
    const cy = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DIRS[d]!.dx;
      const ny = cy + DIRS[d]!.dy;
      if (nx < 0 || nx >= w) continue;
      const j = ny * w + nx;
      if (j === room.entry) nbr[i]![d] = ENTRY;
      else if (local.has(j)) nbr[i]![d] = local.get(j)!;
    }
  }
  {
    // The entry square's only neighbour inside the room model is the door cell.
    const d = local.get(room.door)!;
    const dcx = room.door % w;
    const dcy = (room.door / w) | 0;
    const ecx = room.entry % w;
    const ecy = (room.entry / w) | 0;
    for (let dir = 0; dir < 4; dir++) {
      if (ecx + DIRS[dir]!.dx === dcx && ecy + DIRS[dir]!.dy === dcy) nbr[ENTRY]![dir] = d;
    }
  }

  // Simple deadlocks, by reverse reachability: a floor cell is live when a box on it can
  // be pushed onto a goal (player free to stand anywhere, other boxes ignored). Reverse
  // from each goal: a box at x could have been pushed to y in direction d when the player
  // could stand at x - d. Cells no goal reaches this way are dead; a box on one can never
  // finish, so any state holding one is pruned. This covers corners and goal-less walls.
  const dead = new Uint8Array(n);
  {
    const live = new Uint8Array(n);
    const q = [...goalList];
    for (const g of goalList) live[g] = 1;
    for (let qi = 0; qi < q.length; qi++) {
      const y = q[qi]!;
      for (let d = 0; d < 4; d++) {
        const x = nbr[y]![d ^ 1]!;
        if (x < 0 || x >= n || live[x]) continue;
        const p = nbr[x]![d ^ 1]!;
        if (p < 0) continue; // the player could not have stood there
        live[x] = 1;
        q.push(x);
      }
    }
    for (let i = 0; i < n; i++) if (!live[i]) dead[i] = 1;
  }

  // dist[g][i]: shortest push-path from goal g to cell i over room floor (boxes ignore each other).
  const dist: number[][] = [];
  for (const g of goalList) {
    const dd = new Array<number>(n).fill(Infinity);
    dd[g] = 0;
    const q = [g];
    for (let qi = 0; qi < q.length; qi++) {
      const c = q[qi]!;
      for (let d = 0; d < 4; d++) {
        const j = nbr[c]![d]!;
        if (j < 0 || j >= n || dd[j] !== Infinity) continue;
        dd[j] = dd[c]! + 1;
        q.push(j);
      }
    }
    dist.push(dd);
  }

  // All bijections boxes -> goals, as permutations of goal indices.
  const perms: number[][] = [];
  const used = new Array<boolean>(nb).fill(false);
  const cur: number[] = [];
  const buildPerms = (): void => {
    if (cur.length === nb) {
      perms.push([...cur]);
      return;
    }
    for (let g = 0; g < nb; g++) {
      if (used[g]) continue;
      used[g] = true;
      cur.push(g);
      buildPerms();
      cur.pop();
      used[g] = false;
    }
  };
  buildPerms();

  const heuristic = (bs: number[]): number => {
    let best = Infinity;
    for (const p of perms) {
      let s = 0;
      for (let k = 0; k < nb; k++) {
        const dd = dist[p[k]!]![bs[k]!]!;
        if (dd === Infinity) {
          s = Infinity;
          break;
        }
        s += dd;
      }
      if (s < best) best = s;
    }
    return best;
  };

  const isDead = (bs: number[]): boolean => bs.some((b) => dead[b] === 1);
  const isGoal = (bs: number[]): boolean => bs.every((b) => goalSet.has(b));

  // Reachable squares for the player from `from`, boxes as walls (local indices 0..n).
  const reachable = (from: number, bs: number[]): Uint8Array => {
    const seen = new Uint8Array(n + 1);
    const blocked = new Uint8Array(n + 1);
    for (const b of bs) blocked[b] = 1;
    const q = [from];
    seen[from] = 1;
    for (let qi = 0; qi < q.length; qi++) {
      const c = q[qi]!;
      for (let d = 0; d < 4; d++) {
        const j = nbr[c]![d]!;
        if (j < 0 || seen[j] || blocked[j]) continue;
        seen[j] = 1;
        q.push(j);
      }
    }
    return seen;
  };

  // State storage: boxes, actual player square, pushes so far, heuristic, parent, push that made it.
  const sBoxes: number[][] = [];
  const sPos: number[] = [];
  const sG: number[] = [];
  const sH: number[] = [];
  const sParent: number[] = [];
  const sPushBox: number[] = [];
  const sPushDir: number[] = [];
  const keyOf = (bs: number[], normPos: number): number => {
    let k = normPos;
    for (let t = 0; t < 4; t++) k |= (t < bs.length ? bs[t]! : 63) << (6 + 6 * t);
    return k;
  };
  const best = new Map<number, number>(); // state key -> state id with the lowest g
  const heap: [number, number][] = []; // [f, state id]
  const heapLess = (a: [number, number], b: [number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && sG[a[1]]! > sG[b[1]]!);
  const heapPush = (e: [number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!heapLess(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const heapPop = (): [number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && heapLess(heap[c + 1]!, heap[c]!)) c++;
        if (!heapLess(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };

  const startBoxes = boxes0.map((b) => local.get(b)!).sort((a, b) => a - b);
  const startPos = ENTRY;
  const startReach = reachable(startPos, startBoxes);
  let startNorm = ENTRY;
  for (let i = 0; i <= n; i++) if (startReach[i]) { startNorm = i; break; }
  const h0 = heuristic(startBoxes);
  if (h0 === Infinity || isDead(startBoxes)) return null;
  sBoxes.push(startBoxes);
  sPos.push(startPos);
  sG.push(0);
  sH.push(h0);
  sParent.push(-1);
  sPushBox.push(-1);
  sPushDir.push(-1);
  best.set(keyOf(startBoxes, startNorm), 0);
  heapPush([h0, 0]);

  let goalId = -1;
  let expanded = 0;
  while (heap.length > 0) {
    const [, id] = heapPop();
    const k = keyOf(sBoxes[id]!, (() => {
      const r = reachable(sPos[id]!, sBoxes[id]!);
      for (let i = 0; i <= n; i++) if (r[i]) return i;
      return sPos[id]!;
    })());
    if (best.get(k) !== id) continue; // a better copy of this state was recorded
    const bs = sBoxes[id]!;
    if (isGoal(bs)) {
      goalId = id;
      break;
    }
    expanded++;
    const reach = reachable(sPos[id]!, bs);
    const blocked = new Set(bs);
    for (let bi = 0; bi < bs.length; bi++) {
      const c = bs[bi]!;
      for (let d = 0; d < 4; d++) {
        const pushFrom = nbr[c]![d ^ 1]!; // player stands opposite the push direction
        const dest = nbr[c]![d]!;
        if (pushFrom < 0 || !reach[pushFrom]) continue;
        if (dest < 0 || dest >= n || blocked.has(dest) || dead[dest]) continue;
        const nb2 = bs.slice();
        nb2[bi] = dest;
        nb2.sort((a, b) => a - b);
        if (isDead(nb2)) continue;
        const hh = heuristic(nb2);
        if (hh === Infinity) continue;
        const g2 = sG[id]! + 1;
        const reach2 = reachable(c, nb2);
        let norm = c;
        for (let i = 0; i <= n; i++) if (reach2[i]) { norm = i; break; }
        const k2 = keyOf(nb2, norm);
        const prev = best.get(k2);
        if (prev !== undefined && sG[prev]! <= g2) continue;
        const nid = sBoxes.length;
        sBoxes.push(nb2);
        sPos.push(c);
        sG.push(g2);
        sH.push(hh);
        sParent.push(id);
        sPushBox.push(c);
        sPushDir.push(d);
        best.set(k2, nid);
        heapPush([g2 + hh, nid]);
      }
    }
  }
  if (goalId < 0) return null;

  // Rebuild the moves: walk the parent chain, and for each push re-run the player's walk
  // from the parent's square to the pushing square, then the push step itself (upper-case).
  const chain: number[] = [];
  for (let t = goalId; t > 0; t = sParent[t]!) chain.push(t);
  chain.reverse();
  let moves = '';
  for (const sid of chain) {
    const pid = sParent[sid]!;
    const box = sPushBox[sid]!;
    const dir = sPushDir[sid]!;
    const pushFrom = nbr[box]![dir ^ 1]!;
    // BFS walk from the parent's square to pushFrom, parent's boxes as walls.
    const pBoxes = sBoxes[pid]!;
    const blocked = new Set(pBoxes);
    const prev = new Int32Array(n + 1).fill(-1);
    const stp = new Uint8Array(n + 1);
    const q = [sPos[pid]!];
    prev[sPos[pid]!] = sPos[pid]!;
    for (let qi = 0; qi < q.length; qi++) {
      const c = q[qi]!;
      if (c === pushFrom) break;
      for (let d = 0; d < 4; d++) {
        const j = nbr[c]![d]!;
        if (j < 0 || blocked.has(j) || prev[j] !== -1) continue;
        prev[j] = c;
        stp[j] = d;
        q.push(j);
      }
    }
    if (prev[pushFrom] === -1) return null; // should not happen
    let walk = '';
    for (let c = pushFrom; c !== sPos[pid]!; c = prev[c]!) walk = DIRS[stp[c]!]!.ch + walk;
    moves += walk + DIRS[dir]!.ch.toUpperCase();
  }
  return { moves, endPlayer: globalOf(sPos[goalId]!) };
}

// ---- Whole-level solve: rooms in order, walks between them through the hallway ----
function solve(text: string): string {
  const level = parseLevel(text);
  const rooms = findRooms(level);
  const boxSet = new Set(level.boxes);
  let player = level.player;
  let moves = '';
  for (const room of rooms) {
    const walk = walkMoves(level, boxSet, player, room.entry);
    if (walk === null) return '';
    moves += walk;
    player = room.entry;
    const roomBoxes = level.boxes.filter((b) => room.cellSet.has(b));
    const solved = solveRoom(level, room, roomBoxes, player);
    if (solved === null) return '';
    moves += solved.moves;
    player = solved.endPlayer;
    for (const b of roomBoxes) boxSet.delete(b);
    for (const g of room.goals) boxSet.add(g);
  }
  return moves;
}

for (const id of readdirSync(I).sort()) {
  const moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
  // A push step is written upper-case: report the level's moves and pushes.
  const pushes = (moves.match(/[UDLR]/g) || []).length;
  console.log(`${id}: ${moves.length} moves, ${pushes} pushes${moves.length === 0 ? ' (unsolved)' : ''}`);
}
