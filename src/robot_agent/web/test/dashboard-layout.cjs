// Browser-only replay: intercepted WebSocket never connects to ROS controls.
const assert = require('node:assert/strict');
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || '/home/ruan-x/.cache/ms-playwright-go/1.50.1/package');
const url = process.env.DASHBOARD_TEST_URL || 'http://127.0.0.1:8765';
const initial = [
  {kind:'skill',name:'await_robot_auto_mode',step_id:'auto',owner:'mission'},
  {kind:'task',name:'navigate_to',task_id:'nav',owner:'navigation',payload:{target:'shelf_06'}},
  {kind:'task',name:'search_object',task_id:'see',owner:'perception',payload:{object_query:'red box'}},
  {kind:'task',name:'contact_object',task_id:'arm',owner:'arm',payload:{object_query:'blue dot'}},
];
const skill=(name,id,task,owner='navigation')=>({name,kind:'skill',step_id:id,task_id:task,owner,payload:{target:'shelf_06'}});
const fixture={mission_id:'layout-replay',status:'EXECUTE_STEP',agent_mode:'jev_primary',
  user_text:'go to shelf 06, then find red box, and touch blue dot',current_step_index:4,
  parsed_intent:{task_type:'sequence',constraints:{steps:[{task_type:'navigate_to',target:'shelf_06'},
    {task_type:'search_object',object_query:'red box'},{task_type:'contact_object',object_query:'blue dot'}]}},
  initial_plan:initial,plan:[initial[0],skill('check_localization_health','nav1','nav'),
    skill('resolve_semantic_target','nav2','nav'),skill('goto_pose','nav3','nav'),
    skill('search_at_place','nav4','confirm','perception'),{kind:'task_end',task_id:'confirm'},
    {kind:'task_end',task_id:'nav'},initial[2],initial[3]],
  tasks:[{task_id:'nav',owner:'navigation',task_type:'navigate_to',state:'RUNNING',goal:{task_type:'navigate_to',target:'shelf_06'}},
    {task_id:'confirm',parent_task_id:'nav',owner:'perception',task_type:'search_object',state:'RUNNING',goal:{task_type:'search_object',object_query:'shelf_label_06'}}],
  skill_results:[{step_id:'auto',skill_name:'await_robot_auto_mode',success:true,code:'OK',message:'Robot is in AUTO mode.'},
    {step_id:'nav1',skill_name:'check_localization_health',success:true,code:'OK',message:'Localization verified.'},
    {step_id:'nav2',skill_name:'resolve_semantic_target',success:true,code:'OK',message:'Shelf 06 approach resolved.'},
    {step_id:'nav3',skill_name:'goto_pose',success:true,code:'OK',message:'Goal reached.',metrics:{completion_class:'task_tolerance'}}],
  parser_trace:{accepted_layer:'JEV',plan_review_selected:'jev'},
  jev_trace:{calls:[{call_id:'parse',stage:'command_parse',provider:'api',adopted_decision:'task_graphify_sequence',latency_ms:41,questions:[{id:'sequence_step_count',result:'three'}]},
    {call_id:'bind',stage:'command_bind',provider:'api',adopted_decision:'task_graph_grounded',latency_ms:20,questions:[]},
    {call_id:'review',stage:'plan_review',provider:'api',adopted_decision:'use_jev_plan',latency_ms:28,questions:[{id:'plan_covers_all_subgoals',result:true}]},
    {call_id:'nav-review',stage:'navigation_target_review',task_id:'nav',adopted_decision:'use_known_target',questions:[{id:'decision',result:'use_known_target'}]}]},
};

(async()=>{
 const browser=await chromium.launch({executablePath:process.env.CHROME_BINARY || '/usr/bin/google-chrome',headless:true,args:['--no-sandbox']});
 try {
  for(const [width,height] of [[1920,1080],[1440,1000],[1280,900],[390,844]]) {
   const page=await browser.newPage({viewport:{width,height}});const errors=[];let socket;
   page.on('pageerror',e=>errors.push(String(e)));
   await page.routeWebSocket('**/ws',ws=>{socket=ws;setTimeout(()=>ws.send(JSON.stringify({topic:'/agent/mission_state',data:JSON.stringify(fixture)})),100);});
   await page.goto(url);await page.locator('[data-task-id="arm"]').waitFor();
   assert.equal(await page.locator('.mission-flow-row').count(),4);
   assert.equal(await page.locator('.nested-task').count(),1);
   assert.equal(await page.locator('[data-step-id="nav4"] .execution-badge').textContent(),'実行中');
   assert.equal(await page.locator('[data-task-id="arm"] .execution-badge').textContent(),'未実行');
   assert.equal(await page.locator('.mission-routing .stage-row.done').count(),4);
   await page.getByRole('heading',{name:'Decision Model Routing',exact:true}).waitFor();
   await page.locator('.mission-routing').getByText('navigation_target_review',{exact:true}).waitFor();
   const boxes=await Promise.all(['health','mission','operator'].map(c=>page.locator(`.${c}-column`).boundingBox()));
   if(width>=1200) assert.ok(boxes[0].x < boxes[1].x && boxes[1].x < boxes[2].x);
   else assert.ok(boxes[1].y < boxes[2].y && boxes[2].y < boxes[0].y);
   assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth+2),false,'horizontal overflow');
   await page.screenshot({path:`/tmp/dashboard-layout-${width}.png`,fullPage:true});
   await page.locator('[data-step-id="nav3"] summary').click();
   await page.getByText('task_tolerance',{exact:true}).waitFor();
   const failed=JSON.parse(JSON.stringify(fixture));failed.status='NEEDS_HUMAN';failed.question_for_human='Camera unavailable. Confirm retry or stop.';
   failed.tasks[1].state='WAITING_HUMAN';failed.jev_trace.calls.push({stage:'step_supervision',call_id:'fault',task_id:'confirm',step_id:'nav4',skill_name:'search_at_place',adopted_decision:'ask_human',questions:[{id:'failure_category',result:'camera_data_stale'}]});
   socket.send(JSON.stringify({topic:'/agent/mission_state',data:JSON.stringify(failed)}));
   await page.locator('.mission-blocker').waitFor();
   assert.equal(await page.locator('.mission-routing .stage-row.waiting').count(),1);
   await page.locator('.mission-routing .stage-row.waiting details summary').click();
   await page.getByText('camera_data_stale',{exact:true}).first().waitFor();
   const resolved=JSON.parse(JSON.stringify(failed));resolved.status='DONE';
   resolved.skill_results.push({step_id:'nav4',skill_name:'search_at_place',success:true,code:'OK'});
   socket.send(JSON.stringify({topic:'/agent/mission_state',data:JSON.stringify(resolved)}));
   await page.getByText('Historical decision; no longer awaiting this response. Original decision retained below.',{exact:true}).waitFor();
   assert.equal(await page.locator('.mission-routing .stage-row.waiting').count(),0);
   await page.screenshot({path:`/tmp/dashboard-resolved-${width}.png`,fullPage:true});
   const rejected={...fixture,status:'FAILED_SKILL',initial_plan:[],jev_trace:{calls:[{stage:'plan_review',adopted_decision:'reject'}]}};
   socket.send(JSON.stringify({topic:'/agent/mission_state',data:JSON.stringify(rejected)}));
   await page.waitForFunction(()=>document.querySelectorAll('.mission-routing .stage-row.failed').length===1);
   assert.equal(await page.locator('.mission-flow-row').count(),0);
   const shadow={...fixture,agent_mode:'jev_shadow',parser_trace:{accepted_layer:'Deterministic'},jev_trace:rejected.jev_trace};
   socket.send(JSON.stringify({topic:'/agent/mission_state',data:JSON.stringify(shadow)}));
   await page.getByText('採用: Classic · 計画確定',{exact:true}).waitFor();
   assert.equal(await page.locator('.mission-routing .stage-row.failed').count(),0);
   socket.send(JSON.stringify({topic:'/agent/mission_state',data:JSON.stringify({...failed,status:'CANCELLED'})}));
   await page.waitForFunction(()=>!document.querySelector('.mission-flow-row'));
   assert.deepEqual(errors,[]); console.log(`${width}: hierarchy, pending tasks, review, failure, cancellation, overflow OK`);
   await page.close();
  }
 } finally {await browser.close();}
})().catch(error=>{console.error(error);process.exit(1)});
