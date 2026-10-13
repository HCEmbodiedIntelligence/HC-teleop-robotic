const test=require('node:test');
const assert=require('node:assert/strict');
const {clampWidth,restoreWidths,fitWidths,install}=require('../middleware/core/static/table_columns.js');
test('saved widths are clamped; broken storage falls back to defaults',()=>{
 assert.equal(clampWidth(-1),56);
 assert.equal(clampWidth(1700),1600);
 assert.deepEqual(restoreWidths({getItem:()=>JSON.stringify([100,480])},'x',[64,330]),[100,480]);
 assert.deepEqual(restoreWidths({getItem:()=>'{broken'},'x',[64,330]),[64,330]);
 assert.deepEqual(restoreWidths({getItem:()=>JSON.stringify([null,100])},'x',[64,330]),[64,330]);
});
test('drag changes only one column, maintains total width and supports reset/keyboard',()=>{
 function node(){return {style:{},events:{},attrs:{},classList:{add(){},remove(){}},setAttribute(k,v){this.attrs[k]=v;},addEventListener(k,f){this.events[k]=f;},removeEventListener(k){delete this.events[k];},setPointerCapture(){}};}
 const defaults=[64,330,270,220,100,130,70,128];
 const cols=defaults.map(width=>{const col=node();col.style.width=width+'px';return col;});
 const headers=defaults.map(()=>node());headers.forEach(h=>{h.textContent='话题';h.append=c=>h.handle=c;h.querySelector=()=>h.handle;});
 const table=node(); table.dataset={resizableColumns:'custom'};
 table.querySelectorAll=s=>s==='colgroup col'?cols:s==='thead th'?headers:[];
 table.parentElement={clientWidth:1062};
 table.ownerDocument={createElement:node,body:node()};
 install(table);
 const h=headers[1].handle;
 h.events.pointerdown({button:0,clientX:100,pointerId:1,preventDefault(){}});
 h.events.pointermove({clientX:300});
 assert.equal(cols[1].style.width,'420px');assert.equal(cols[0].style.width,'64px');
 assert.equal(table.style.width,'1262px');
 h.events.pointerup();assert.equal(h.events.pointermove,undefined);
 h.events.keydown({key:'ArrowLeft',preventDefault(){}});
 assert.equal(cols[1].style.width,'410px');
 h.events.dblclick();assert.equal(cols[1].style.width,'220px');
 assert.equal(table.style.width,'1062px');
});

test('content widths fit viewport without shrinking status columns; overflow respects minimums',()=>{
 const desired=[64,600,500,300,100,130,70,128];
 const minimums=[64,220,180,170,100,130,70,128];
 const fitted=fitWidths(desired,minimums,1300,[1,2,3]);
 assert.equal(fitted.reduce((a,b)=>a+b,0),1300);
 for(const i of [0,4,5,6,7]) assert.equal(fitted[i],desired[i]);
 assert.deepEqual(fitWidths(desired,minimums,500,[1,2,3]),minimums);
 const expanded=fitWidths(desired,minimums,2400,[1,2,3]);
 assert.equal(expanded.reduce((a,b)=>a+b,0),2400);
 for(const i of [0,4,5,6,7]) assert.equal(expanded[i],desired[i]);
});
