# Repository rules

## Publication boundary

- Keep stable, user-facing helpers in `tools/`.
- Keep captures, research scripts, tuning runs, protocols, and one-off analysis in `experiments/`.
- Treat every file under `experiments/` as local-only. Keep `/experiments/` covered by `.gitignore`.
- Do not mention, link, or advertise local experiments in public documentation.
- Never force-add ignored content.

## Commit boundary

1. Inspect `git status --short` before staging.
2. Stage only explicit production files needed for the current change.
3. Inspect `git diff --cached --stat` and `git diff --cached` before committing.
4. Confirm the staged set contains no captures, generated outputs, `DESTILLATION.md`, or `experiments/` content.

## house rules

1. DON'T TOUCH README.MD for god's sake! Only do, when user asks for it!
2. this is a public repo. Don't commit shit.
3. Use ./experiments for experiments.
4. this repo is ONLY for dump1030 and some helpful tools around 1030 MHz radar shit. If something is about dump1090, don't put it here (or apply rule #3)