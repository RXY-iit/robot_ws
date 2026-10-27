const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const ts = require('typescript');
const vm = require('node:vm');

const source = fs.readFileSync(path.join(__dirname, '../src/lib/missionLogic.ts'), 'utf8');
const output = ts.transpileModule(source, {compilerOptions: {module: ts.ModuleKind.CommonJS}}).outputText;
const exportsObject = {};
vm.runInNewContext(output, {exports: exportsObject});

test('task owner, not legacy skill mapping, controls agent status', () => {
  const status = exportsObject.computeAgentStatus({status: 'DONE', current_step_index: 1,
    plan: [{name: 'check_localization_health', owner: 'navigation', kind: 'skill'},
           {name: 'navigate_to', owner: 'navigation', kind: 'task_end'}]}, null, null, [], null);
  assert.equal(status['Navigation Agent'].state, 'done');
  assert.equal(status['Health / Recovery Agent'].state, '');
});

test('normal unmet goal is not shown as execution failure', () => {
  const status = exportsObject.computeAgentStatus({status: 'DONE', tasks: [{owner: 'perception',
    task_type: 'search_object', state: 'COMPLETED', goal_satisfied: false, outcome: 'UNMET'}]}, null, null, [], null);
  assert.equal(status['Perception Agent'].state, 'waiting');
});
