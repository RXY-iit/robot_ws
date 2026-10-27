// All WebSocket traffic is intercepted: no robot or mode commands leave this browser.
const assert = require('node:assert/strict');
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || '/home/ruan-x/.cache/ms-playwright-go/1.50.1/package');
(async()=>{
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox']});
 try {
  for(const width of [1440,390]) {
   const page=await browser.newPage({viewport:{width,height:1000}});const errors=[],out=[];let ws;
   page.on('pageerror',e=>errors.push(String(e)));
   await page.routeWebSocket('**/ws',socket=>{ws=socket;socket.onMessage(m=>out.push(JSON.parse(m)));});
   await page.goto(process.env.DASHBOARD_TEST_URL || 'http://127.0.0.1:8765');
   await page.getByText('connected',{exact:true}).waitFor();
   const send=(topic,value)=>ws.send(JSON.stringify({topic,data:JSON.stringify(value)}));
   const state={mission_id:'m1',status:'NEEDS_HUMAN',user_text:'go back ...',question_for_human:'incomplete_command_placeholder',
     intent:{task_type:'ask_human',constraints:{ambiguity:{choices:[{label:'Cancel',command:'cancel'}, {label:'Cancel all',command:'cancel all'}]}}}};
   send('/agent/mission_state',{...state,intent:{...state.intent,constraints:{ambiguity:{choices:[
     ...state.intent.constraints.ambiguity.choices,{label:'Set pose home',command:'set pose home'}]}}}});
   await page.locator('.choice-panel').getByRole('button',{name:/Set pose home/}).waitFor();
   assert.equal(await page.locator('.choice-panel button').count(),2,'Dynamic non-cancel choices must remain');
   send('/agent/mission_state',state);
   await page.locator('.choice-panel').waitFor();
   await page.waitForFunction(()=>document.querySelectorAll('.choice-panel button').length===1);
   assert.equal(await page.locator('.choice-panel button').count(),1);
   const cancelName='キャンセル（全部） / Cancel all';
   await page.locator('.choice-panel').getByRole('button',{name:cancelName,exact:true}).click();
   await page.waitForFunction(()=>!document.querySelector('.choice-panel'));
   assert.equal(out.filter(x=>x.type==='cancel').length,1);
   assert.equal(out.filter(x=>x.type==='command').length,0);
   const wait={mission_id:'m2',wait_id:'w2',step_id:'auto',skill_name:'await_robot_auto_mode',actions:['set_auto'],auto_resume:true};
   send('/agent/mission_state',{...state,mission_id:'m2',user_text:'go back 20cm',human_wait:wait});
   await page.getByRole('button',{name:'AUTO に変更して続行',exact:true}).click();
   await page.waitForTimeout(100);
   assert.deepEqual(out.find(x=>x.type==='mission_control'),{type:'mission_control',command:{action:'set_auto',mission_id:'m2',wait_id:'w2'}});
   send('/agent/mission_state',{...state,status:'CANCELLED'});
   send('/agent/mission_events',{type:'mission_cancelled',mission_id:'m1'});
   send('/agent/mission_state',state);
   await page.waitForTimeout(100);
   assert.equal(await page.getByRole('button',{name:'AUTO に変更して続行',exact:true}).count(),1,'Late old state must not clear new wait');
   await page.screenshot({path:`/tmp/human-controls-${width}.png`,fullPage:true});
   // Command-area cancel must have the exact same effect and transport as ask-human cancel.
   assert.equal(await page.getByRole('button',{name:cancelName,exact:true}).count(),2);
   await page.getByRole('button',{name:cancelName,exact:true}).first().click();
   await page.waitForFunction(()=>!document.querySelector('.choice-panel'));
   send('/agent/mission_state',{...state,mission_id:'m2',human_wait:wait});
   await page.waitForTimeout(100);
   assert.equal(await page.locator('.choice-panel').count(),0,'Cancelled mission must not return');
   assert.deepEqual(errors,[]);
   assert.equal(out.filter(x=>x.type==='cancel').length,2);
   assert.equal(out.filter(x=>x.type==='command').length,0);
   assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
   console.log(`${width}: identical cancel-all controls, scoped AUTO approval, late cancellation and state isolation passed`);
   await page.close();
  }
 } finally {await browser.close();}
})().catch(error=>{console.error(error);process.exit(1)});
