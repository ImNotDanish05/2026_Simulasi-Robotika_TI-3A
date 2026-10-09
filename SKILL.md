---
name: danish-ai-agent-context
description: Maintains a single, always-current project context file (DANISH_AI_AGENT.md) so any AI agent can understand this project by reading ONE file, instead of re-scanning the whole codebase every session. MUST be used at the START of every task in this project — before exploring any other files, check whether DANISH_AI_AGENT.md exists and read it first. If it doesn't exist, run the "First Scan" procedure to create it. MUST be used again after finishing any new feature, bugfix, refactor, or architecture change — update the file before considering the task done. Triggers: starting work on an unfamiliar project, "where do I start", onboarding a new agent, or any request to add/change code in this repo.
---

# Danish AI Agent Context

This skill maintains a single file, `DANISH_AI_AGENT.md`, at the project root. It's not documentation for humans — it's the context an AI agent needs so it doesn't have to re-read the entire codebase every time a new session starts. This follows the same pattern as `AGENTS.md`/`CLAUDE.md`, which have become an industry standard, just customized so the agent maintains it automatically.

Why this actually saves cost: without this file, every new session has to re-read the folder structure, package manifest, README, and guess at coding conventions from examples — that can burn thousands of tokens on the same rediscovery, every single time. With one concise file, the agent just reads that and gets to work.

## Step 1 — Orientation (REQUIRED at the start of EVERY task, no matter how small)

1. Check whether `DANISH_AI_AGENT.md` exists at the project root.
2. **If it exists** → read it first, BEFORE exploring any other files. Don't re-scan the whole project unless the file itself says a relevant area is undocumented.
3. **If it doesn't exist** → don't start working yet. Run Step 2 first.

## Step 2 — First Scan (only if the file doesn't exist yet)

Scan at a high level first — do NOT read the full contents of every file:
- Folder structure (1-2 levels deep, skip `node_modules/`, `.git/`, `dist/`, `vendor/`, and similar)
- Package manifest (`package.json`, `pyproject.toml`, etc.) to learn the stack and dependencies
- Existing README
- Config files (linter, formatter, CI, env example)
- A handful of representative code files to pick up the conventions actually in use (not the "textbook" ones)

From this, work out: the project's purpose, the exact stack (not just "Python" but "Python 3.12 + Poetry, not pip"), the build/run/test commands that actually work, what each folder is for, conventions that differ from defaults, and areas that shouldn't be touched (secrets, generated files, etc.).

Write a draft `DANISH_AI_AGENT.md` using the template in Step 3, then **flag the draft to the user for a quick review** before treating it as ground truth — research shows auto-generated context files that skip human review can actually hurt accuracy and increase inference cost, because they tend to restate things the agent could already infer on its own.

## Step 3 — Format of `DANISH_AI_AGENT.md`

**Golden rule: keep it SHORT.** Target 30-50 lines for small-to-medium projects, hard cap ~150 lines. This file gets sent with every single prompt — every unnecessary line dilutes the model's attention away from the lines that actually matter.

Test every line with: *"If this line were removed, would the agent make a mistake it wouldn't otherwise make?"* If not, delete it.

```markdown
# DANISH_AI_AGENT.md
> This file is for AI agents, not humans. Update after finishing every task. Read this BEFORE exploring other files.

## Overview
1-2 sentences: what this project is, who it's for.

## Stack
Language + version, framework, package manager, EXACTLY (e.g. "pnpm, not npm").

## Commands
- Install: `...`
- Run dev: `...`
- Test: `...`
- Build: `...`

## Structure map (top-level only, 1 line per folder)
- `src/api/` — REST endpoints
- `src/workers/` — async jobs
- ...

## Conventions that differ from defaults / can't be inferred
- e.g. "all errors are wrapped in AppError, never throw a plain Error"
- e.g. "tests use vitest, not jest, even though package.json still has leftover jest config"

## DO NOT touch
- `legacy/` — not migrated yet, don't change without asking
- `.env*` files, `*.lock` files

## Known gotchas
- Anything that trips the agent up if it isn't warned first.

## Changelog (newest first, 1 line per entry, NOT a diff)
- 2026-09-04: Added /refund endpoint, see src/api/refund.ts
```

Don't copy-paste README content in here. If something's already covered in the README, just point to it ("see README section X") instead of duplicating it — duplication means paying for the same tokens twice on every prompt.

## Step 4 — Update AFTER finishing work (not optional)

After finishing a feature, bugfix, or refactor, before telling the user the task is done:
1. Update the relevant sections (Stack/Commands/Structure map/Conventions) if anything changed — **edit in place**, don't just append below.
2. Add one changelog line: what changed + why. Not the diff itself.
3. If any old instruction is no longer true (e.g. folder structure changed), **fix or remove it**, don't leave it — stale documentation is more dangerous than no documentation, because it confidently sends the agent the wrong way.
4. Re-check the file's length. If it's crept past the cap, trim before adding — e.g. merge old changelog lines into one, or drop gotchas that no longer apply.

## Extra cost-saving practices (from research)

- **Don't duplicate context files.** If this project also has `AGENTS.md`/`CLAUDE.md`/`.cursorrules` for other tools, don't re-fill them — just add one line pointing to `DANISH_AI_AGENT.md` as the source of truth.
- **Keep the top of the file stable** (no timestamps or frequently-changing info at the very top) — most LLM providers now offer prompt caching, where identical content across prompts is billed at a fraction of the price, but even a small change to the prefix invalidates the cache and you pay full price again.
- **Don't connect every tool/MCP server at once** if the task doesn't need them — unused tool definitions still cost tokens on every turn, sometimes tens of thousands of tokens before the model has even processed the user's instruction.
- **Read code files selectively.** For large files, search/grep for the relevant part first instead of reading the whole file top to bottom.
- **For very long sessions**, summarize older parts of the conversation into a compact note if it's genuinely gotten too long — but don't rush to do this; with prompt caching active, keeping full history is sometimes cheaper and more accurate than repeatedly re-summarizing.

## Edge cases

- **Monorepo / multiple sub-projects:** it's fine to add extra `DANISH_AI_AGENT.md` files inside each package, scoped to that package. The root file can just be an index pointing to each one.
- **Conflicts with an existing context file** (`AGENTS.md`, etc.): don't create two sources of truth. Pick one as canonical, make the other a one-line pointer.