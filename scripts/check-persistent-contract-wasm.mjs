import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
const modulePath = process.argv[2];
assert(modulePath, "usage: node scripts/check-persistent-contract-wasm.mjs <persistent-contract.wasm>");
const module = await WebAssembly.compile(await readFile(modulePath));
for (let replay = 0; replay < 3; replay++) {
  const instance = await WebAssembly.instantiate(module, {});
  assert.equal(instance.exports.run_persistent_contract(), 12);
}
console.log("Persistent adapter WASM contract: 12 shared fixtures passed across 3 independent replays.");
