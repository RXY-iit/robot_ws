const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const ts = require('typescript');
const vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname, '../src/lib/missionView.ts'), 'utf8');
const api = {};
vm.runInNewContext(ts.transpileModule(source, {compilerOptions: {module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020}}).outputText, {exports: api});
const plain = x => JSON.parse(JSON.stringify(x));
const task = (id, name, owner) => ({kind:'task', task_id:id, name, owner});
const skill = (id, taskId, name) => ({kind:'skill', step_id:id, task_id:taskId, name});
function mission() {
  return {mission_id:'m1', status:'EXECUTE_STEP', current_step_index:2,
    initial_plan:[task('t1','navigate_to','navigation'), task('t2','search_object','perception'), task('t3','set_arm_pose','arm')],
    plan:[skill('s1','t1','check_localization_health'),skill('s2','t1','goto_pose'),skill('p1','child','search_at_place'),
      {kind:'task_end',task_id:'child'}, {kind:'task_end',task_id:'t1'},task('t2','search_object','perception'),task('t3','set_arm_pose','arm')],
    tasks:[{task_id:'t1',task_type:'navigate_to',owner:'navigation',state:'RUNNING'},
      {task_id:'child',parent_task_id:'t1',task_type:'search_object',owner:'perception',state:'RUNNING'}],
    skill_results:[{step_id:'s1',step_index:0,skill_name:'check_localization_health',success:true,code:'OK'},
      {step_id:'s2',step_index:1,skill_name:'goto_pose',success:true,code:'OK'}]};
}
test('conditional false branch is visibly skipped, not waiting or completed motion',()=>{
  const m=mission(); m.status='DONE';
  m.tasks.push({task_id:'t2',task_type:'search_object',state:'SKIPPED',outcome:'CONDITION_FALSE'});
  assert.equal(api.executionGraph(m)[1].status,'skipped');
});
test('stable mission roots preserve pending tasks and nested domain ownership',()=>{
  const graph=api.executionGraph(mission());
  assert.deepEqual(plain(graph.map(n=>n.id)),['t1','t2','t3']);
  assert.deepEqual(plain(graph[0].entries.map(n=>n.id)),['s1','s2','child']);
  assert.equal(graph[0].entries[2].entries[0].status,'active');
  assert.equal(graph[1].status,'pending');
  assert.equal(graph[1].entries.length,0);
});
test('refresh of old telemetry reconstructs hierarchy without events',()=>{
  const m=mission(); delete m.initial_plan;
  assert.deepEqual(plain(api.executionGraph(m).map(n=>n.id)),['t1','t2','t3']);
});
test('retry results stay on one step, newest result wins',()=>{
  const m=mission(); m.skill_results.unshift({step_id:'s2',step_index:1,skill_name:'goto_pose',success:false,code:'FAILED_TIMEOUT'});
  const node=api.executionGraph(m)[0].entries[1];
  assert.equal(node.attempts.length,2); assert.equal(node.status,'done');
});
test('task NOT_FOUND differs from execution failure and cancellation',()=>{
  const m=mission();m.status='DONE';m.tasks[1]={...m.tasks[1],state:'COMPLETED',goal_satisfied:false};
  m.skill_results.push({step_id:'p1',skill_name:'search_at_place',success:true,code:'OK',metrics:{goal_satisfied:false}});
  assert.equal(api.executionGraph(m)[0].entries[2].status,'unmet');
  assert.equal(api.executionGraph(m)[0].entries[2].entries[0].status,'unmet');
  m.status='CANCELLED'; assert.equal(api.executionGraph(m)[1].status,'cancelled');
});
test('failed current skill without result is not pending, cancelled never active',()=>{
  const m=mission();m.status='FAILED_SKILL';
  assert.equal(api.executionGraph(m)[0].entries[2].entries[0].status,'failed');
  assert.equal(api.executionGraph(m)[0].status,'failed');
  m.status='CANCELLED';assert.equal(api.executionGraph(m)[0].entries[2].entries[0].status,'cancelled');
});
test('review rejection is not accepted merely because candidate plan exists',()=>{
  const m=mission();m.agent_mode='jev_primary';delete m.initial_plan;
  m.jev_trace={calls:[{stage:'plan_review',adopted_decision:'reject'}]};m.status='FAILED_SKILL';
  assert.equal(api.planAccepted(m,[]),false);
  m.agent_mode='jev_shadow';m.status='EXECUTE_STEP';assert.equal(api.planAccepted(m,[]),true);
  m.initial_plan=[]; assert.equal(api.planAccepted(m,[]),false);
});
test('original parse remains distinct from selected classic intent',()=>{
  const m={mission_id:'m',parsed_intent:{task_type:'rotate_relative'},intent:{task_type:'move_relative'}};
  assert.equal(api.parsedIntent(m,[]).task_type,'rotate_relative');
  delete m.parsed_intent;
  assert.equal(api.parsedIntent(m,[{mission_id:'other',type:'intent_parsed',payload:{intent:{task_type:'wrong'}}}]).task_type,'move_relative');
});
test('old trace origin correlates only to this mission',()=>{
  const m={mission_id:'m',jev_trace:{calls:[{call_id:'c',stage:'step_supervision'}]}};
  const events=[{mission_id:'other',payload:{trace:{call_id:'c'},step_index:9}},
    {mission_id:'m',payload:{trace:{call_id:'c'},step_index:2,skill:'goto_pose'}}];
  assert.equal(api.correlatedCalls(m,events)[0].step_index,2);
  assert.equal(api.correlatedCalls(m,events)[0].skill_name,'goto_pose');
});
test('repeated standalone skills use separate plan positions',()=>{
  const m={status:'EXECUTE_STEP',current_step_index:1,initial_plan:[{name:'wait'},{name:'wait'}],plan:[{name:'wait'},{name:'wait'}]};
  assert.deepEqual(plain(api.executionGraph(m).map(n=>n.index)),[0,1]);
});

test('resolved supervision is history, not a permanent human wait',()=>{
  const call={stage:'step_supervision',adopted_decision:'ask_human_enable_auto',step_id:'s1',step_index:0};
  const m=mission(); m.status='NEEDS_HUMAN'; m.current_step_index=0; m.skill_results=[];
  assert.equal(api.stageDecisionStatus(call,m),'waiting');
  m.skill_results=[{step_id:'s1',success:true,code:'OK'}];
  assert.equal(api.stageDecisionStatus(call,m),'done');
  m.skill_results=[];m.status='DONE';assert.equal(api.stageDecisionStatus(call,m),'done');
  m.status='CANCELLED';assert.equal(api.stageDecisionStatus(call,m),'cancelled');
  m.status='FAILED_SKILL';assert.equal(api.stageDecisionStatus(call,m),'failed');
  assert.equal(api.stageDecisionStatus(call),'waiting','original judgment is immutable');
});

test('another step waiting or Shadow suggestion does not revive old human request',()=>{
  const call={stage:'navigation_plan_review',adopted_decision:'ask_human',step_id:'s1'};
  const m=mission();m.status='NEEDS_HUMAN';m.skill_results=[];
  assert.equal(api.stageDecisionStatus(call,m),'done');
  m.current_step_index=0;m.agent_mode='jev_shadow';
  assert.equal(api.stageDecisionStatus(call,m),'done');
});

test('grounded task graph is not ask_human even while a domain task waits',()=>{
  const m=mission(); m.status='NEEDS_HUMAN';
  for (const decision of ['task_graph_grounded','task_graph_jev_sequence','task_graph_jev_single_step','structure_human_clarified']) {
    const call={stage:'command_bind',adopted_decision:decision};
    assert.equal(api.stageDecisionStatus(call),'done');
    assert.equal(api.stageDecisionStatus(call,m),'done');
  }
  for (const decision of ['ask_human','ask_human:invalid_span','ask_human_inconsistent_parse','call_recovery_agent']) {
    assert.equal(api.stageDecisionStatus({adopted_decision:decision},m),'waiting');
  }
});

test('a later accepted review resolves only the same task and stage history',()=>{
  const call={call_id:'old',stage:'perception_command_parse',adopted_decision:'ask_human',task_id:'child'};
  const m=mission(); m.status='NEEDS_HUMAN';
  const accepted={call_id:'new',stage:call.stage,adopted_decision:'complete',task_id:'t2'};
  m.jev_trace={calls:[call,accepted]};
  assert.equal(api.stageDecisionStatus(call,m),'waiting');
  accepted.task_id='child';
  assert.equal(api.stageDecisionStatus(call,m),'done');
  assert.equal(api.stageDecisionStatus(call),'waiting');
});
