export function hash3(x, y, z, seed = 1) {
  let h = Math.imul(x ^ Math.imul(y + 374761393, 668265263) ^ Math.imul(z + seed * 144269, 2246822519), 3266489917);
  h ^= h >>> 13;
  h = Math.imul(h, 1274126177);
  return ((h ^ (h >>> 16)) >>> 0) / 4294967295;
}

export function valueNoise(x, y, z, seed = 1) {
  const xi = Math.floor(x);
  const yi = Math.floor(y);
  const zi = Math.floor(z);
  const tx = smooth(x - xi);
  const ty = smooth(y - yi);
  const tz = smooth(z - zi);
  let v = 0;

  for (let dz = 0; dz <= 1; dz++) {
    for (let dy = 0; dy <= 1; dy++) {
      for (let dx = 0; dx <= 1; dx++) {
        const wx = dx ? tx : 1 - tx;
        const wy = dy ? ty : 1 - ty;
        const wz = dz ? tz : 1 - tz;
        v += hash3(xi + dx, yi + dy, zi + dz, seed) * wx * wy * wz;
      }
    }
  }

  return v;
}

export function fractureNoise(mode, p, seed = 1) {
  const x = p.x;
  const y = p.y;
  const z = p.z;

  if (mode === 'ridge') {
    const n = valueNoise(x * 2.4 + 13, y * 2.4, z * 2.4 - 7, seed);
    return 1 - Math.abs(n * 2 - 1);
  }

  if (mode === 'bands') {
    const wave = Math.sin((x * 2.2 + y * 0.55 - z * 1.4 + seed) * Math.PI);
    return 0.5 + 0.5 * wave;
  }

  if (mode === 'turbulence') {
    let amp = 0.55;
    let freq = 1.2;
    let total = 0;
    for (let i = 0; i < 4; i++) {
      total += Math.abs(valueNoise(x * freq, y * freq, z * freq, seed + i * 19) * 2 - 1) * amp;
      amp *= 0.5;
      freq *= 2.15;
    }
    return Math.min(1, total);
  }

  return valueNoise(x * 1.8, y * 1.8, z * 1.8, seed);
}

function smooth(t) {
  return t * t * (3 - 2 * t);
}
