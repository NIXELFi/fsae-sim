// Plan view of the selected course for the launch screen: centreline, cones,
// start gate and the direction of travel. Drawn once per course change.

const GOLD = "#FFC627";

export function drawCoursePlan(canvas, track) {
  const dpr = Math.min(devicePixelRatio || 1, 2);
  const cw = canvas.clientWidth || 320, ch = canvas.clientHeight || 180;
  canvas.width = Math.floor(cw * dpr);
  canvas.height = Math.floor(ch * dpr);
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, cw, ch);

  const pts = track.center;
  if (!pts?.length) return;
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (const [x, y] of pts) {
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
  }
  const pad = 18;
  const k = Math.min((cw - pad * 2) / Math.max(1, maxX - minX), (ch - pad * 2) / Math.max(1, maxY - minY));
  const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
  const P = (x, y) => [cw / 2 + (x - cx) * k, ch / 2 - (y - cy) * k];

  // Course width as a soft band under the line, so the plan reads as a road.
  ctx.lineCap = "round"; ctx.lineJoin = "round";
  ctx.beginPath();
  for (let i = 0; i < pts.length; i += 2) {
    const [x, y] = P(pts[i][0], pts[i][1]);
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  }
  if (track.closed) ctx.closePath();
  ctx.strokeStyle = "rgba(255,255,255,0.10)";
  ctx.lineWidth = Math.max(3, (track.width || 4) * k);
  ctx.stroke();
  ctx.strokeStyle = "rgba(232,236,240,0.75)";
  ctx.lineWidth = 1.5;
  ctx.stroke();

  // Cones, as the tiny orange dots they are from the air.
  if (track.cones?.length) {
    ctx.fillStyle = "rgba(255,120,40,0.75)";
    for (const c of track.cones) {
      const [x, y] = P(c.x, c.y);
      ctx.fillRect(x - 0.75, y - 0.75, 1.5, 1.5);
    }
  }

  // Start gate and direction arrow.
  const [sx, sy] = P(pts[0][0], pts[0][1]);
  const h = track.heading?.[0] ?? 0;
  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(-h);
  ctx.fillStyle = GOLD;
  ctx.beginPath();
  ctx.moveTo(7, 0); ctx.lineTo(-4, 4.5); ctx.lineTo(-2, 0); ctx.lineTo(-4, -4.5);
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  // Scale bar: 50 m, bottom right.
  const bar = 50 * k;
  if (bar > 20 && bar < cw / 2) {
    const x1 = cw - pad, x0 = x1 - bar, y = ch - 10;
    ctx.strokeStyle = "rgba(255,255,255,0.45)";
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x0, y); ctx.lineTo(x1, y);
    ctx.moveTo(x0, y - 3); ctx.lineTo(x0, y + 3); ctx.moveTo(x1, y - 3); ctx.lineTo(x1, y + 3);
    ctx.stroke();
    ctx.fillStyle = "rgba(255,255,255,0.55)";
    ctx.font = "9px ui-monospace, monospace";
    ctx.textAlign = "right";
    ctx.fillText("50 m", x1, y - 5);
  }
}
