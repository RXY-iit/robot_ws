// Mock transport only: exercising scoped human replies must never send robot commands.
const assert = require('node:assert/strict');
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || '/home/ruan-x/.cache/ms-playwright-go/1.50.1/package');
(async () => {
  const {preview} = await import('vite');
  const server = await preview({preview: {host: '127.0.0.1', port: 0}});
  let browser;
  try {
    browser = await chromium.launch({executablePath: '/usr/bin/google-chrome', headless: true, args: ['--no-sandbox']});
    for (const width of [1440, 390]) {
      const page = await browser.newPage({viewport: {width, height: 1000}});
      const sent = [], errors = [];
      let socket;
      page.on('pageerror', error => errors.push(String(error)));
      await page.routeWebSocket('**/ws', ws => {socket = ws; ws.onMessage(msg => sent.push(JSON.parse(msg)));});
      await page.route('**/api/config', route => route.fulfill({json: {quick_commands: [], mission_test_commands: []}}));
      await page.goto(`http://127.0.0.1:${server.httpServer.address().port}`);
      await page.getByText('connected', {exact: true}).waitFor();
      const label = "Replace 'box' object with 'suitcase' (changes target)";
      const state = {mission_id: 'perception-test', status: 'NEEDS_HUMAN', user_text: 'home then find box',
        question_for_human: 'No approved exact detector for box',
        human_wait: {mission_id: 'perception-test', wait_id: 'choice1', skill_name: 'search_object', auto_resume: false,
          actions: ['perception_replace_0'], action_labels: {perception_replace_0: label}}};
      socket.send(JSON.stringify({topic: '/agent/mission_state', data: JSON.stringify(state)}));
      const choice = page.getByRole('button', {name: label, exact: true});
      await choice.click();
      await page.waitForTimeout(100);
      assert.deepEqual(sent.find(x => x.type === 'mission_control'), {type: 'mission_control', command: {
        action: 'perception_replace_0', mission_id: 'perception-test', wait_id: 'choice1'}});
      assert.equal(sent.filter(x => x.type === 'command').length, 0);
      await page.screenshot({path: `/tmp/perception-controls-${width}.png`, fullPage: true});
      assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth + 1), false);
      await page.locator('.choice-panel').getByRole('button', {name: /Cancel all/}).click();
      await page.waitForTimeout(100);
      assert.equal(sent.filter(x => x.type === 'cancel').length, 1);
      assert.deepEqual(errors, []);
      console.log(`${width}: scoped substitution, no new command, cancel all and layout passed`);
      await page.close();
    }
  } finally {
    if (browser) await browser.close();
    await new Promise(resolve => server.httpServer.close(resolve));
  }
})().catch(error => {console.error(error); process.exit(1);});
