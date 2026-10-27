// Read-only fixtures; all WebSocket traffic is intercepted, never sent to ROS.
const assert = require('node:assert/strict');
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || '/home/ruan-x/.cache/ms-playwright-go/1.50.1/package');
const fixture = {schema_version:1, generated_at:new Date().toISOString(),sequence:1,summaries:[],capabilities:[],domains:[],
 readiness:{generated_at:new Date().toISOString(),scope:'monitored_prerequisites_only',
 groups:[['base','Communication and safety','READY'],['navigation','Navigation baseline','BLOCKED'],['perception','Visual acquisition','READY'],['arm','Arm control interface','WAITING']]
 .map(([id,title,status])=>({id,title,status,summary:status==='READY'?'Monitored prerequisites are currently satisfied':'Waiting for fresh evidence',
 first_ready_at:'2026-10-12T04:59:05Z',ready_since:status==='READY'?'2026-10-12T04:59:05Z':null,
 status_since:'2026-10-12T05:00:00Z',checks:[{id:id==='navigation'?'gicp_pose':'agent_heartbeat',status:status==='BLOCKED'?'STALE':'OBSERVED',summary:'Receiver evidence'}],
 notes:['MANUAL is not a startup failure. Readiness does not authorize motion.']})),
 supplemental:[{id:'sim_sync:rgb_depth',status:'WARN',summary:'Nearest stamp skew 0.317s'},
 {id:'perception_runtime',status:'IDLE',summary:'No inference yet'}]}};
const recovery={env:'sim',launch_profile:'nav2',incidents:[],recently_closed:[],playbooks:{active:20},
 ignored_warnings:[{key:'simulation_performance',playbook_id:'PB-SIM-RTF-LOW',active:true,summary:'RTF warning'},
 {key:'camera',active:false,summary:'Not currently triggered'}]};
(async()=>{
 const {preview}=await import('vite');
 const server=await preview({preview:{host:'127.0.0.1',port:0}});
 const url=`http://127.0.0.1:${server.httpServer.address().port}`;
 let browser;
 try {
  browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox']});
  for(const [width,route] of [[1440,'/'],[390,'/'],[1440,'/health']]) {
   const page=await browser.newPage({viewport:{width,height:1000}});let socket;const errors=[],commands=[];
   page.on('pageerror',e=>errors.push(String(e)));
   await page.routeWebSocket('**/ws',ws=>{socket=ws;ws.onMessage(m=>commands.push(m));
    setTimeout(()=>{fixture.generated_at=new Date().toISOString();ws.send(JSON.stringify({topic:'/agent/health_dashboard_state',data:JSON.stringify(fixture)}));
      ws.send(JSON.stringify({topic:'/agent/recovery_state',data:JSON.stringify(recovery)}));},100);});
   await page.route('**/api/config',r=>r.fulfill({json:{quick_commands:[],mission_test_commands:[],default_command:''}}));
   await page.goto(`${url}${route}`);
   await page.locator('.readiness-ready').first().waitFor();
   assert.equal(await page.locator('.readiness-item').count(),4);
   assert.equal(await page.locator('.readiness-ready').count(),2);
   await page.getByRole('heading',{name:'Readiness by Capability',exact:true}).waitFor();
   if(route==='/') {
    await page.getByText('Snoozed warnings (2)',{exact:true}).waitFor();
    assert.equal(await page.getByRole('button',{name:'Restore alerts',exact:true}).count(),2);
   }
   await page.locator('.readiness-blocked summary').click();
   await page.locator('.readiness-blocked').getByText('gicp_pose',{exact:true}).waitFor();
   assert.ok((await page.locator('.readiness-blocked').textContent()).includes('13:59:05'));
   await page.locator('.readiness-context summary').click();
   await page.locator('.readiness-context').getByText('perception_runtime',{exact:true}).waitFor();
   await page.screenshot({path:`/tmp/readiness-${width}-${route==='/'?'agent':'health'}.png`,fullPage:true});
   assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth+2),false);
   if(width===390) {
    await page.waitForTimeout(11000);
    assert.equal(await page.locator('.readiness-ready').count(),0);
    assert.equal(await page.locator('.readiness-stale').count(),4);
    fixture.generated_at=new Date().toISOString();
    socket.send(JSON.stringify({topic:'/agent/health_dashboard_state',data:JSON.stringify(fixture)}));
    await page.locator('.readiness-ready').first().waitFor();
   }
   assert.deepEqual(errors,[]);assert.deepEqual(commands,[]);
   console.log(`${route} ${width}: scope, evidence, JST time, layout and stale handling passed`);
   await page.close();
  }
 }finally{if(browser) await browser.close();await new Promise(resolve=>server.httpServer.close(resolve));}
})().catch(e=>{console.error(e);process.exit(1)});
