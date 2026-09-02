# Agent conventions

## AGENT-001 — Deterministic checks before agent judgment

- Encode mechanically checkable properties as executable checks.

## AGENT-003 — Separate execution from orchestration

- Keep the development loop independent of its local, CI, or hosted orchestration adapter.

## AGENT-004 — The harness defines completion

- The harness owns the completion gates; agents propose and repair changes.

## AGENT-005 — Integration is its own workspace

- Combine and validate independently produced changes in a dedicated integration workspace.

## AGENT-006 — Prefer mechanical discovery before semantic search

- Derive relationships from paths, names, metadata, or indexes before searching semantically.

## AGENT-007 — Run cheap validation before expensive validation

- Run required checks from narrowest and cheapest to broadest and most expensive; stop at the first failure.

## AGENT-008 — Revalidate downward after broader-scope fixes

- After fixing a broad validation failure with production-code changes, restart at the narrowest affected layer.

## AGENT-009 — Delegate one bounded capability per implementation run

- Give each delegated implementation run one independently verifiable capability slice.
- Use a validated, pinned task packet when the surrounding cross-component protocol requires one; do not invent missing contract data or widen the assigned scope.
- Only one active implementation run may own an overlapping path or behavioral scope.
- Report undeclared prerequisites, drift, overlap, or inconsistent delegated inputs to the delegating caller or coordination layer for replanning.
- Distinguish completing a partial slice from satisfying the broader convention.

## AGENT-010 — Apply progressive composition to agent execution

- Resolve execution-layer choices to `PRINCIPLE-006` and progressive verification to `PRINCIPLE-003`; AGENT-010 is the agent-category pointer and adds no second copy of those policies.
- Gather deterministic evidence first, preserve it across attempts, use focused validation while iterating, and escalate only when cheaper execution stops producing useful information or runtime/environment feedback is inherently required.
