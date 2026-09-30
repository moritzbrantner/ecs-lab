import assert from 'node:assert/strict';
import {readFile, writeFile} from 'node:fs/promises';
const [modulePath, outputPath] = process.argv.slice(2);
assert(modulePath && outputPath, "usage: node scripts/record-rotating-consumers-wasm.mjs <ecs-web-demo.wasm> <output.json>");
const {instance} = await WebAssembly.instantiate(await readFile(modulePath), {});
const e = instance.exports;
const rows=[];
assert.equal(e.physics_demo_max_steps(), 600);
let fixed;
for(let step=0; step<=600; step++) {
  const count=e.physics_demo_body_count(step);
  assert.equal(count,54,`playground frame ${step}`);
  const boxes=[];
  for(let body=0; body<count; body++) {
    const get = name => e[`physics_demo_${name}`](body,step);
    const p=['x','y','z'].map(axis=>get(`position_${axis}`));
    const q=['x','y','z','w'].map(axis=>get(`orientation_${axis}`));
    const omega=['x','y','z'].map(axis=>get(`angular_velocity_${axis}`));
    const half=['x','y','z'].map(axis=>get(`half_extent_${axis}`));
    const bounds=['min','max'].map(end=>['x','y','z'].map(axis=>get(`broad_${end}_${axis}`)));
    assert([...p,...q,...omega,...half,...bounds.flat()].every(Number.isFinite));
    assert(half.every(v=>v>0));
    boxes.push({id:get('entity_id'), fixed:get('is_fixed'), mass:get('mass_units'), restitution:get('restitution_milli'),friction:get('friction_milli'),p,q,omega,half,bounds});
  }
  const fixedNow=boxes.filter(b=>b.fixed);
  if(step===0) fixed=fixedNow;
  else assert.deepEqual(fixedNow,fixed,`fixed playground walls at ${step}`);
  rows.push({scene:'playground',step,boxes,overlaps:e.physics_demo_overlap_count(step)});
}
assert.equal(e.physics_tower_demo_max_steps(),480);
assert.equal(e.physics_tower_demo_body_count(),32);
let floor;
for(let step=0; step<=240; step++) {
  const vertices=[];
  for(let body=0;body<32;body++) {
    const corners=[];
    for(let vertex=0;vertex<8;vertex++) {
      const xyz=['x','y','z'].map(axis=>e[`physics_tower_demo_vertex_${axis}`](body,vertex,step));
      assert(xyz.every(Number.isFinite));
      corners.push(xyz);
    }
    vertices.push(corners);
  }
  if(step===0) {
    floor=vertices[0];
    assert(floor.flat().some(v=>v!==0));
  } else assert.deepEqual(vertices[0],floor,`tower floor/default-failure sentinel at ${step}`);
  rows.push({scene:'tower',step,vertices,spinning:e.physics_tower_demo_spinning_bodies(step),sampled_events:e.physics_tower_demo_sampled_events(step),tail_contacts:e.physics_tower_demo_tail_contacts(step)});
}
await writeFile(outputPath,JSON.stringify(rows)+'\n');
console.log(`Actual WASM consumer projection: ${rows.length} valid frames; playground fixed walls and tower floor sentinel retained.`);
