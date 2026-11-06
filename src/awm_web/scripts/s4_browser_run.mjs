/**
 * Scripted browser run of S2 → plan → S4 playback (G1.6) with headless Chromium (Playwright), against a running
 * awm_api that serves the built front-end. Runs inside awm_dev; credentials only from the environment.
 *
 *   AWM_UI_URL=http://127.0.0.1:8048 AWM_UI_USER=<engineer> AWM_UI_PASSWORD=<password> AWM_JOB=<job id> \
 *   AWM_SHOT_DIR=<folder for png> [AWM_TRAVEL_SPEED_MM_S=3] [AWM_PLAN=1] \
 *   PLAYWRIGHT_BROWSERS_PATH=<browser folder> node scripts/s4_browser_run.mjs
 *
 * Steps: log in; open S2; (AWM_PLAN=1) enter the travel speed and press "Plan all seams", wait for the run;
 * open S4; check the "Kinematic only — collision not checked" banner; play the first and the last weld segment and
 * take a screenshot of each (s4_seg1.png, s4_seg2.png …). Exit 1 on any failed check or page error.
 */
import { mkdirSync } from 'node:fs';
import { join } from 'node:path';
import { chromium } from 'playwright';

const env = (k, d) => {
  const v = process.env[k] ?? d;
  if (v === undefined || v === '') throw new Error(`${k} is not set`);
  return v;
};
const url = env('AWM_UI_URL', 'http://127.0.0.1:8048');
const job = env('AWM_JOB');
const shots = env('AWM_SHOT_DIR');
const speed = process.env.AWM_TRAVEL_SPEED_MM_S ?? '';
mkdirSync(shots, { recursive: true });

const browser = await chromium.launch({ args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: 1280, height: 720 } });
const errors = [];
page.on('pageerror', (e) => errors.push(`pageerror: ${e.message}`));
page.on('console', (m) => m.type() === 'error' && errors.push(`console: ${m.text()}`));
const check = (ok, what) => {
  if (!ok) throw new Error(`check failed: ${what}`);
  console.log(`ok  ${what}`);
};

try {
  await page.goto(`${url}/#/jobs`);
  await page.fill('#u', env('AWM_UI_USER'));
  await page.fill('#p', env('AWM_UI_PASSWORD'));
  await page.click('button.btn');
  await page.waitForSelector(`text=${job}`, { timeout: 15000 });
  await page.goto(`${url}/#/jobs/${job}`);
  await page.waitForSelector('canvas', { timeout: 30000 });
  await page.waitForTimeout(1500);
  await page.screenshot({ path: join(shots, 's2_seams.png') });

  if (process.env.AWM_PLAN === '1') {
    if (speed) await page.fill('input[placeholder="planner default (Assumption)"]', speed);
    await page.click('text=Plan all seams');
    await page.waitForSelector('.run-succeeded, .run-failed', { timeout: 240000 });
    check(await page.locator('.run-succeeded').count(), 'plan run succeeded');
  }

  await page.goto(`${url}/#/jobs/${job}/sim`);
  await page.waitForSelector('[data-testid=sim-overlay]', { timeout: 60000 });
  const banner = await page.locator('[data-testid=kinematic-banner]').textContent();
  check(banner?.includes('Kinematic only — collision not checked'), 'banner "Kinematic only — collision not checked"');
  // H-CAD head (20 MB STL) and UR meshes: wait for the network to settle
  await page.waitForLoadState('networkidle', { timeout: 120000 });
  await page.waitForTimeout(2000);
  const source = await page.locator('header .muted').last().textContent();
  console.log(`source: ${source}`);
  const msgs = await page.locator('[data-testid=speed-msg]').allTextContents();
  console.log(`speed per arc: ${msgs.join(' | ')}`);

  // segment starts from the timeline: scrub, then play a little of each weld segment
  const max = Number(await page.locator('[data-testid=scrubber]').getAttribute('max'));
  const scrub = async (t) =>
    page.locator('[data-testid=scrubber]').evaluate((el, v) => {
      const set = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set;
      set.call(el, String(v));
      el.dispatchEvent(new Event('input', { bubbles: true }));
    }, t);
  const seams = [];
  for (const [i, frac] of [
    [1, 0.2],
    [2, 0.7],
  ]) {
    await scrub(max * frac);
    await page.click('[data-testid=play]');
    await page.waitForTimeout(1500);
    await page.click('[data-testid=play]');
    const overlay = await page.locator('[data-testid=sim-overlay]').textContent();
    console.log(`segment ${i}: ${overlay}`);
    seams.push(/\bS\d{2,3}\b/.exec(overlay ?? '')?.[0]);
    check(/segment (weld|positioner_weld)/.test(overlay ?? ''), `segment ${i} is a weld segment`);
    await page.screenshot({ path: join(shots, `s4_seg${i}.png`) });
  }
  await page.selectOption('[data-testid=focus]', 'cell');
  await page.waitForTimeout(800);
  await page.screenshot({ path: join(shots, 's4_cell.png') });
  check(seams[0] && seams[1] && seams[0] !== seams[1], `two different seams played (${seams.join(', ')})`);
  check(errors.length === 0, `no page errors${errors.length ? ': ' + errors.join(' | ') : ''}`);
} catch (e) {
  console.error(String(e));
  if (errors.length) console.error(errors.join('\n'));
  await page.screenshot({ path: join(shots, 'failure.png') }).catch(() => undefined);
  process.exitCode = 1;
} finally {
  await browser.close();
}
