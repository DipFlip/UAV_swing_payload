# Claude Code Instructions

## Workflow

- After completing any request that changes the simulation code, automatically commit and push to `origin main` without asking. This triggers the GitHub Pages deployment.
- Always increment the version number in `frontend/index.html` when pushing changes. The version appears in three places and **all three must be updated together** to the same version:
  1. `style.css?v=X.Y` (cache bust — if not updated, browsers serve stale CSS)
  2. `main.js?v=X.Y` (cache bust — if not updated, browsers serve stale JS)
  3. `<div id="hud-version">vX.Y</div>` (visible version in HUD)
