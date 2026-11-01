---
sidebar_position: 4
---

# AI Usage

We recommend not using AI where you can avoid it. You learn the stack by working through it yourself, and that's the whole point of being on the team. We also know it's sometimes the fast option, so this page is about using it the right way.

Your first point of contact for help should always be the leads, not Claude. Ask us first.

## Install Claude Code

Follow the official install guide: [Claude Code overview](https://docs.claude.com/en/docs/claude-code/overview).

WashU students can get Claude Edu, which includes Claude Code, for free through [WashU IT](https://it.washu.edu/claude-for-education/).

:::note Danforth students
Complete the [AI Literacy Canvas module](https://ai.washu.edu/ai-literacy/) first. That gets the standard Claude Edu subscription fully subsidized. Then sign in at claude.ai with your WashU email using **Continue with SSO**, and check that **Claude Edu** shows under your name in the bottom-left. That's how you know you're in the WashU workspace and not a personal account.
:::

<!-- TODO: screenshot of the claude.ai sidebar with "Claude Edu" under the user's name
![Claude Edu in the claude.ai sidebar](/img/ai-usage/claude-edu-sidebar.png "Claude Edu")
-->

## Make sure the repo context is set up

`wuair_v2` ships files that tell Claude how our stack works, so it doesn't have to grep through the whole repo every time you ask it something. That makes it faster and much less likely to make things up. Make sure they're there before you start:

- `CLAUDE.md` at the repo root: the ground rules Claude reads at the start of every session (config vs source edits, shared messages, the CAN naming, and so on).
- `.claude/skills/`: step-by-step instructions for specific jobs (`deploy-systemd`, `record-rosbag`).
- `.claude/reference/env-vars.md`: every `WUAIR_*` variable.
- `graphify-out/graph.json`: a code graph of the whole stack. `CLAUDE.md` tells Claude to query it before reading source files, so it can answer "where does X happen" without opening fifty files.

The graph is built with [graphify](https://github.com/Graphify-Labs/graphify). Install it (the package name really does have two y's), then set it up for Claude Code:

<!-- TODO: confirm `pipx install graphifyy` on a fresh Linux / Mac / WSL machine -->

```bash
pipx install graphifyy
graphify install --platform claude    # adds the /graphify skill to Claude Code
graphify hook install                 # keeps the graph updated on every commit, checkout and merge
```

Check that it works from the root of `wuair_v2`:

```bash
graphify query "where is the CAN interface brought up"
```

When you change code, refresh the graph. This only parses code, so it's free:

```bash
graphify update .
```

Changes to docs or PDFs need a full re-extract, which does cost tokens. Run it inside Claude Code:

```text
/graphify --update
```

## Install Ponytail and set it to full

[Ponytail](https://github.com/DietrichGebert/ponytail) is a Claude Code plugin that makes Claude write only the code a task actually needs. Without it, Claude loves generating a pile of random bullshit (extra options, wrappers, "for later" code) that clogs up our development and that someone has to review and delete.

Install it from inside Claude Code:

```text
/plugin marketplace add DietrichGebert/ponytail
/plugin install ponytail@ponytail
```

Then set it to full:

```text
/ponytail full
```

<!-- TODO: screenshot of the Ponytail install output in Claude Code
![Installing Ponytail](/img/ai-usage/ponytail-install.png "Installing Ponytail")
-->

:::warning Node.js has to be on your PATH
Ponytail's hooks run on Node.js. If `node` isn't on your PATH, the plugin fails silently: it looks installed, but it isn't actually doing anything. Check before you trust it:

```bash
node --version
```

If you see the `[PONYTAIL]` badge in Claude Code's status line, it's active.
:::

<!-- TODO: screenshot of the [PONYTAIL] badge in the Claude Code status line
![Ponytail status line badge](/img/ai-usage/ponytail-statusline.png "Ponytail badge")
-->

## Review everything it writes

Please review any code Claude writes and understand every line before you commit it. What looks like a minor change can be a big difference in behavior, and on a car that matters. If you can't explain a line in your PR, don't commit it. Ask Claude, or better, ask a lead.

Anyway, XOXO, enjoy coding.
