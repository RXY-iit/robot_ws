// Fixture-only browser validation. Every WebSocket is intercepted; no ROS commands.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || '/home/ruan-x/.cache/ms-playwright-go/1.50.1/package');
const output = process.env.RECOVERY_TEST_OUTPUT || '/tmp/recovery-environment-ui';
const base = () => ({env:'sim', generated_at:new Date().toISOString(), launch_profile:'nav2', incidents:[], recently_closed:[],
  playbooks:{active:20,inactive:1}, last_cycle:{result:'ENVIRONMENT_UNAVAILABLE',pending:[]},
  environment:{state:'OFFLINE',summary:'No local Isaac process; downstream recovery suspended.',recovery_allowed:false,evidence:{process_alive:false},suppressed_items:['gicp_score','delivery_safety']},
  startup_units:[{id:'nav',label:'Nav2 and costmaps',source:'src/nav_pkg/launch/navigation.launch.py',status:'ENVIRONMENT_UNAVAILABLE',recovery:'manual',
    playbooks:['PB-NODE-DOWN'],evidence:{lifecycle_bt_navigator:'STALE'},limitation:'No controlled group start yet.'}],
  execution:{enabled:true,active:[],finished:[],operator_requests:[]}});
const incident = (key,kind) => ({id:key,key,title:key,status:'CONFIRMED',outcome:'CONFIRMED',handling_class:kind,
  playbook_id:kind==='unknown_investigation'?null:'PB-NODE-DOWN',playbook_no:50,severity:'degraded',priority:2,layer:'ros_delivery',
  roots:[{item:key,status:'STALE',summary:'No new output; evidence retained.'}],root_items:[key],consequences:[],
  diagnosis_path:{upstream_checks:[],playbook_steps:[]},plan:null,reason:'Fixture evidence',reopen_count:0});
(async()=>{
  fs.mkdirSync(output,{recursive:true});
  const {preview}=await import('vite');
  const server=await preview({preview:{host:'127.0.0.1',port:0}});
  const url=`http://127.0.0.1:${server.httpServer.address().port}`;
  let browser;
  try {
    browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox']});
    for (const width of [1600,390]) {
      const page=await browser.newPage({viewport:{width,height:1000}});
      let socket; const errors=[],commands=[];
      page.on('pageerror',e=>errors.push(String(e)));
      await page.route('**/api/config',r=>r.fulfill({json:{quick_commands:[],mission_test_commands:[],default_command:''}}));
      await page.routeWebSocket('**/ws',ws=>{socket=ws;ws.onMessage(m=>commands.push(m));
        setTimeout(()=>ws.send(JSON.stringify({topic:'/agent/recovery_state',data:JSON.stringify(base())})),50);});
      await page.goto(url);
      const panel=page.locator('.recovery-panel');
      await panel.getByText('Environment / 環境状態: OFFLINE',{exact:true}).waitFor();
      assert.equal(await panel.locator('.operator-request').count(),0);
      await panel.getByText('Startup coverage / 起動項目 (1)',{exact:true}).click();
      await panel.getByText('No controlled group start yet.',{exact:true}).waitFor();
      await page.screenshot({path:`${output}/offline-${width}.png`,fullPage:true});
      const running=base(); running.lifecycle='RUNNING'; running.auto_approve=false;
      running.environment={...running.environment,state:'RUNNING',summary:'Fresh advancing clock.',recovery_allowed:true,suppressed_items:[]};
      running.incidents=[incident('mode_switch','known_recovery'),incident('camera','unknown_investigation'),incident('physical_estop','safety_event')];
      running.incidents[0].last_execution={state:'ABORTED',message:'Operator rejected restart_component'};
      running.execution.operator_requests=[{id:'fixture-approve',kind:'confirm_action',title:'Start mode_switch',text:{en:'Start in MANUAL; no duplicate process.'},
        buttons:['approve','reject'],waiting_sec:0,last_check:[],step:{primitive:'restart_component',risk:'A1',args:{component:'mode_switch'}},note:'',
        action_context:{scope:'Start only the missing mode switch',opens_terminal:true,
          launch_context:{source:'Quick Start launch snapshot'},node_check:{detail:'Missing: /nav_mode_switch_node'},
          command_template:['ros2','run','nav_pkg','nav_mode_switch_node.py'],preconditions:['raw_cmd_vel_zero_fresh']},
        done_when:[{probe:'node_alive',args:{component:'mode_switch'}}]}];
      socket.send(JSON.stringify({topic:'/agent/recovery_state',data:JSON.stringify(running)}));
      await panel.getByText('Environment / 環境状態: RUNNING',{exact:true}).waitFor();
      await panel.getByText('Last run: ABORTED · Operator rejected restart_component',{exact:true}).waitFor();
      await panel.getByText('Action scope and verification / 操作範囲と検証',{exact:true}).click();
      await panel.getByText(/Missing: \/nav_mode_switch_node/).waitFor();
      await panel.getByRole('button',{name:'Approve / 承認',exact:true}).click();
      await page.waitForTimeout(50);
      assert.equal(commands.length,1);
      assert.ok(commands[0].includes('fixture-approve') && commands[0].includes('approve'));
      const auto=panel.getByRole('checkbox',{name:/PB 自动批准/});
      assert.equal(await auto.isChecked(),false);
      await auto.click();
      await page.waitForTimeout(50);
      assert.ok(commands[1].includes('set_auto_approve') && commands[1].includes('true'));
      running.auto_approve=true;
      socket.send(JSON.stringify({topic:'/agent/recovery_state',data:JSON.stringify(running)}));
      await page.waitForTimeout(50);
      assert.equal(await auto.isChecked(),true);

      await panel.getByText('Investigator is not connected. Evidence retained; no speculative restart.',{exact:true}).waitFor();
      assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth+2),false);
      await page.screenshot({path:`${output}/running-${width}.png`,fullPage:true});
      running.generated_at='2000-01-01T00:00:00Z';
      socket.send(JSON.stringify({topic:'/agent/recovery_state',data:JSON.stringify(running)}));
      await panel.getByText('Environment / 環境状態: UNKNOWN (stale)',{exact:true}).waitFor();
      assert.ok(await panel.getByRole('button',{name:'Approve / 承認',exact:true}).isDisabled());
      assert.deepEqual(errors,[]);
      console.log(`${width}: environment, coverage, classification, intercepted approval, stale lockout, layout passed`);
      await page.close();
    }
  } finally {
    if(browser) await browser.close();
    await new Promise(resolve=>server.httpServer.close(resolve));
  }
})().catch(e=>{console.error(e);process.exitCode=1});
