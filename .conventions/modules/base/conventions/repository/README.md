# Repository conventions

## REPO-001 — Repository structure encodes agent-relevant relationships

- Prefer layouts whose relationships are mechanically derivable from paths, hierarchy, naming, or local metadata.

## REPO-002 — More specific conventions override broader conventions

- On conflict, use the narrowest applicable rule; non-conflicting broader rules remain in force.
- Precedence: repository rule → deepest technology scope → parent scopes → general convention → principle.

## REPO-003 — Template decisions are executable

- Encode template defaults in working configuration, scripts, structure, dependencies, tests, and examples.

## REPO-004 — Validate templates from a fresh instance

- A template is complete only when a fresh instance can install, start, test, and build without undeclared local state.

## REPO-005 — Templates include one small vertical slice

- Include one thin, real end-to-end feature that demonstrates the intended architecture.

## REPO-006 — Dogfood the template workflow

- Maintain templates through the same structure, commands, tests, and agent workflow given to downstream projects.

## REPO-007 — Do not preinstall speculative architecture

- Include dependencies and abstractions only when they are intentional template defaults.

## REPO-008 — Templates expose a canonical validation interface

- Make the commands for development, focused tests, broader validation, and build mechanically obvious.

## REPO-009 — Use conventional roots for durable agent-authored project knowledge

Unless a repository explicitly overrides them, use:

- `CONTEXT.md` for the concise domain glossary and project-level domain overview;
- `docs/domain/` for richer domain-first hierarchical knowledge;
- `docs/adr/` for consequential architectural decisions;
- `docs/specs/` for canonical human-readable implementation specs;
- `docs/reviews/` for durable review history when review persistence is enabled.

These are repository-layout defaults, not requirements that every repository create every directory. Create a durable artifact only when the corresponding knowledge exists. Runtime agent state, ticket queues, run evidence, and derived catalog/profile resolution do not belong in these roots.

## REPO-010 — Keep TODOs machine-discoverable and actionable

- Developer-authored TODOs are legitimate follow-up work that agents may later enumerate, implement, or turn into issues.
- Use `TODO: <actionable description>` when no issue exists and `TODO(#123): <actionable description>` when the work is already tracked.
- A linked issue is optional; vague markers such as `TODO fix this` or unexplained `FIXME` comments are not an acceptable durable backlog.

## REPO-011 — Verify only meaningful file permissions

- Directly executable scripts and tools have the executable bit when the target platform uses it; ordinary source/config files should not accidentally become executable.
- Sensitive private-key material created locally uses restrictive permissions appropriate to the platform.
- Do not normalize permission metadata that has no portable semantic meaning.

## REPO-012 — Treat symlinks as explicit filesystem boundaries

- Deterministic traversal does not recursively follow symlinks by default.
- When symlinks are intentionally supported, targets stay inside the declared repository/workspace boundary unless the command explicitly permits external targets.
- Hashing, copying, cleanup, and generation distinguish the link from its target rather than silently traversing it.

## REPO-013 — Keep path casing portable

- Imports and references use the exact on-disk path casing.
- A repository must not contain case-colliding tracked entries such as `User.ts` and `user.ts`.
- Perform case-only renames explicitly through Git rather than relying on the host filesystem's case sensitivity.
