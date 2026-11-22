import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { execFile } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');
const BUILD_DIR = path.join(ROOT, 'build');
const SIM_EXE = path.join(BUILD_DIR, 'opennear_sim.exe');
const TEST_EXE = path.join(BUILD_DIR, 'test_opennear.exe');
const DATA_JS = path.join(__dirname, 'opennear_dashboard_data.js');
const DASH_HTML = path.join(__dirname, 'dashboard.html');
const PORT = 38080;

function parseDashboardJs() {
  const raw = fs.readFileSync(DATA_JS, 'utf8');
  const idx = raw.indexOf('{');
  const last = raw.lastIndexOf('}');
  return JSON.parse(raw.slice(idx, last + 1));
}

const server = http.createServer((req, res) => {
  const u = new URL(req.url, `http://127.0.0.1:${PORT}`);

  if (u.pathname === '/api/run_tests') {
    execFile(TEST_EXE, [], { cwd: BUILD_DIR, timeout: 15000 }, (err, stdout, stderr) => {
      res.setHeader('Content-Type', 'application/json; charset=utf-8');
      if (err) {
        res.end(JSON.stringify({ ok: false, error: err.message, stderr, stdout }));
        return;
      }
      res.end(JSON.stringify({ ok: true, stdout }));
    });
    return;
  }

  if (u.pathname === '/api/run_sim') {
    const drift = u.searchParams.get('drift_scale') || '1.0';
    const jitter = u.searchParams.get('hw_jitter_ns') || '35';
    const blackout = u.searchParams.get('blackout_sfs') || '10';
    const jam = u.searchParams.get('jam_mask') || '0x0000';
    const kill = u.searchParams.get('kill_mode') || 'g';

    const args = [
      '--drift-scale', drift,
      '--hw-jitter-ns', jitter,
      '--blackout-sfs', blackout,
      '--jam-mask', jam,
      '--kill-mode', kill
    ];

    execFile(SIM_EXE, args, { cwd: BUILD_DIR, timeout: 15000 }, (err, stdout, stderr) => {
      res.setHeader('Content-Type', 'application/json; charset=utf-8');
      if (err) {
        res.end(JSON.stringify({ ok: false, error: err.message, stderr }));
        return;
      }
      try {
        const data = parseDashboardJs();
        res.end(JSON.stringify({ ok: true, stdout, data }));
      } catch (parseErr) {
        res.end(JSON.stringify({ ok: false, error: parseErr.message, stdout }));
      }
    });
    return;
  }

  if (u.pathname === '/opennear_dashboard_data.js' || u.pathname === '/tools/opennear_dashboard_data.js') {
    res.setHeader('Content-Type', 'application/javascript; charset=utf-8');
    res.end(fs.readFileSync(DATA_JS, 'utf8'));
    return;
  }

  res.setHeader('Content-Type', 'text/html; charset=utf-8');
  res.end(fs.readFileSync(DASH_HTML, 'utf8'));
});

server.listen(PORT, '127.0.0.1', () => {
  console.log(`[OpenNear Chaos Workbench] Listening on http://127.0.0.1:${PORT}`);
});
