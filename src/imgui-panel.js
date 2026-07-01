const COLORS = {
  bg: 'rgba(18, 21, 26, 0.88)',
  panel: '#171b21',
  border: '#39414b',
  text: '#e8eef3',
  dim: '#9faab4',
  accent: '#49b6b0',
  warn: '#e6a14b',
  track: '#2b3139',
};

export class ImGuiPanel {
  constructor(canvas, state, callbacks) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.state = state;
    this.callbacks = callbacks;
    this.items = [];
    this.pointer = { x: 0, y: 0, down: false, capture: null };
    this.scale = 1;
    this.needsRefracture = false;

    canvas.addEventListener('pointerdown', (event) => this.onPointer(event, true));
    window.addEventListener('pointermove', (event) => this.onPointer(event, this.pointer.down));
    window.addEventListener('pointerup', (event) => this.onPointer(event, false, true));
  }

  resize(width, height, scale) {
    this.scale = scale;
    this.canvas.width = Math.floor(width * scale);
    this.canvas.height = Math.floor(height * scale);
    this.canvas.style.width = `${width}px`;
    this.canvas.style.height = `${height}px`;
    this.draw();
  }

  draw(stats = this.state.stats) {
    const ctx = this.ctx;
    const s = this.scale;
    ctx.setTransform(s, 0, 0, s, 0, 0);
    ctx.clearRect(0, 0, this.canvas.width / s, this.canvas.height / s);
    this.items = [];

    let x = 16;
    let y = 16;
    const w = 292;
    const row = 28;
    const h = 570;

    roundRect(ctx, x, y, w, h, 8, COLORS.bg, COLORS.border);
    ctx.font = '600 14px Inter, system-ui, sans-serif';
    ctx.fillStyle = COLORS.text;
    ctx.fillText('Fracture Lab', x + 14, y + 25);
    ctx.font = '12px Inter, system-ui, sans-serif';
    ctx.fillStyle = COLORS.dim;
    ctx.fillText(`Jolt ${stats.physics}  S:${stats.shards}  Bodies:${stats.bodies}  Joints:${stats.joints}`, x + 14, y + 44);

    y += 64;
    y = this.combo(x + 14, y, w - 28, 'Fracture', ['voronoi', 'ridge', 'bands', 'turbulence'], 'noise') + 5;
    y = this.combo(x + 14, y, w - 28, 'Joint type', ['fixed', 'locked', 'distance'], 'jointType') + 5;
    y = this.slider(x + 14, y, w - 28, 'Target shards', 'shards', 8, 160, 1) + 4;
    y = this.slider(x + 14, y, w - 28, 'Cluster size', 'clusterSize', 2, 24, 1) + 4;
    y = this.slider(x + 14, y, w - 28, 'Joint slack', 'jointSoftness', 0, 0.35, 0.01) + 4;
    y = this.slider(x + 14, y, w - 28, 'Load safety', 'loadSafety', 1.1, 6, 0.05) + 4;
    y = this.slider(x + 14, y, w - 28, 'Failure delay', 'failureDelay', 0.05, 1, 0.05) + 4;
    y = this.slider(x + 14, y, w - 28, 'Impact force', 'impactForce', 4, 80, 1) + 4;
    y = this.slider(x + 14, y, w - 28, 'Impact radius', 'impactRadius', 0.15, 2.0, 0.05) + 7;

    const buttonW = (w - 36) / 2;
    this.button(x + 14, y, buttonW, row, 'Refracture', () => this.callbacks.refracture());
    this.button(x + 22 + buttonW, y, buttonW, row, 'Reset sim', () => this.callbacks.reset());
    y += row + 10;

    this.checkbox(x + 14, y, 'Anchor base shards', 'anchorBase');
    y += 24;
    this.checkbox(x + 14, y, 'Show joint graph', 'showJoints');
    y += 27;

    ctx.fillStyle = COLORS.dim;
  }

  slider(x, y, w, label, key, min, max, step) {
    const ctx = this.ctx;
    const value = this.state[key];
    ctx.fillStyle = COLORS.text;
    ctx.font = '12px Inter, system-ui, sans-serif';
    ctx.fillText(`${label}: ${formatValue(value, step)}`, x, y + 10);

    const tx = x;
    const ty = y + 16;
    const tw = w;
    const th = 8;
    const t = (value - min) / (max - min);
    roundRect(ctx, tx, ty, tw, th, 4, COLORS.track);
    roundRect(ctx, tx, ty, tw * t, th, 4, COLORS.accent);
    ctx.beginPath();
    ctx.arc(tx + tw * t, ty + th / 2, 7, 0, Math.PI * 2);
    ctx.fillStyle = COLORS.text;
    ctx.fill();

    this.items.push({ type: 'slider', key, x: tx, y: ty - 8, w: tw, h: 24, min, max, step });
    return y + 34;
  }

  combo(x, y, w, label, values, key) {
    const ctx = this.ctx;
    ctx.fillStyle = COLORS.text;
    ctx.font = '12px Inter, system-ui, sans-serif';
    ctx.fillText(label, x, y + 10);

    const segW = w / values.length;
    for (let i = 0; i < values.length; i++) {
      const active = this.state[key] === values[i];
      roundRect(ctx, x + i * segW, y + 18, segW - 4, 24, 5, active ? COLORS.accent : COLORS.track, COLORS.border);
      ctx.fillStyle = active ? '#07110f' : COLORS.text;
      ctx.fillText(values[i], x + i * segW + 8, y + 34);
      this.items.push({ type: 'combo', key, value: values[i], x: x + i * segW, y: y + 18, w: segW - 4, h: 24 });
    }
    return y + 42;
  }

  button(x, y, w, h, label, action) {
    const ctx = this.ctx;
    roundRect(ctx, x, y, w, h, 5, COLORS.panel, COLORS.border);
    ctx.fillStyle = COLORS.text;
    ctx.font = '600 12px Inter, system-ui, sans-serif';
    ctx.fillText(label, x + 12, y + 18);
    this.items.push({ type: 'button', action, x, y, w, h });
  }

  checkbox(x, y, label, key) {
    const ctx = this.ctx;
    roundRect(ctx, x, y, 18, 18, 4, this.state[key] ? COLORS.accent : COLORS.track, COLORS.border);
    if (this.state[key]) {
      ctx.strokeStyle = '#07110f';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(x + 4, y + 9);
      ctx.lineTo(x + 8, y + 13);
      ctx.lineTo(x + 14, y + 5);
      ctx.stroke();
    }
    ctx.fillStyle = COLORS.text;
    ctx.font = '12px Inter, system-ui, sans-serif';
    ctx.fillText(label, x + 26, y + 13);
    this.items.push({ type: 'checkbox', key, x, y, w: 170, h: 20 });
  }

  onPointer(event, down, up = false) {
    const rect = this.canvas.getBoundingClientRect();
    this.pointer.x = event.clientX - rect.left;
    this.pointer.y = event.clientY - rect.top;
    this.pointer.down = down && !up;

    const active = this.pointer.capture || this.items.find((item) => hit(item, this.pointer.x, this.pointer.y));
    if (!active) return;

    event.preventDefault();
    if (active.type === 'slider' && down) {
      this.pointer.capture = active;
      const t = Math.min(1, Math.max(0, (this.pointer.x - active.x) / active.w));
      const raw = active.min + t * (active.max - active.min);
      this.state[active.key] = Math.round(raw / active.step) * active.step;
      this.draw();
    }

    if (up) {
      if (active.type === 'button') active.action();
      if (active.type === 'combo') {
        this.state[active.key] = active.value;
        this.callbacks.refracture();
      }
      if (active.type === 'checkbox') {
        this.state[active.key] = !this.state[active.key];
        if (active.key === 'anchorBase') this.callbacks.refracture();
        this.draw();
      }
      if (active.type === 'slider' && !['impactForce', 'impactRadius', 'failureDelay'].includes(active.key)) {
        this.callbacks.refracture();
      }
      this.pointer.capture = null;
    }
  }
}

function roundRect(ctx, x, y, w, h, r, fill, stroke = null) {
  ctx.beginPath();
  ctx.roundRect(x, y, w, h, r);
  ctx.fillStyle = fill;
  ctx.fill();
  if (stroke) {
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 1;
    ctx.stroke();
  }
}

function hit(item, x, y) {
  return x >= item.x && x <= item.x + item.w && y >= item.y && y <= item.y + item.h;
}

function formatValue(value, step) {
  return step < 1 ? value.toFixed(2) : String(Math.round(value));
}
